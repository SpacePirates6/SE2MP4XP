using System;
using System.Collections.Generic;
using System.Reflection;
using HarmonyLib;
using Keen.VRage.Core;
using Keen.VRage.DCS.Accessors;
using Keen.VRage.DCS.Annotations;
using Keen.VRage.Library.Mathematics;
using Keen.VRage.Physics.Character;
using Keen.VRage.Physics.Data;

namespace SpinProbe;

/// <summary>
/// Puts every physics job that touches pose data into the rotating frame, by wrapping its DCS invocation stub.
/// <para>Why stubs: the per-body wrappers (HknpWorld.SetBodyInfo / DriveBodyTo / GetBodyInfo, VRage.Physics\...\Havok\Core\
/// HknpWorld.cs:66-117) are [AggressiveInlining], so a Harmony patch on them would be skipped wherever they were inlined;
/// and the batch natives HkWorld.SetBodiesInfo / GetBodiesInfo read and write ECS memory directly through the stub's data
/// pointers (HavokSessionComponent.cs:2001, :2140). Stubs are called through a function pointer by Scene.RunJob
/// (VRage.DCS\...\Scenes\Scene.cs:2445), never inlined.</para>
/// <para>Stub layout (HavokSessionComponent.cs:1988-2170, and every generated stub): dataPointers[slot] = pointer to the first
/// row of that slot's data, ((int*)dataPointers[-1])[slot] = its stride, slots in [InvocationStubData] order with slot 0 =
/// DEntity. The stub advances dataPointers[slot] in place while it loops, so bases are captured in the prefix.</para>
/// <para>Rule, per slot access (InvocationStubData.DataAccess):
///  ReadOnly  - before: world→frame (save the original); after: restore the original bytes.
///  Writable  - before: world→frame; after: frame→world.
///  Write     - after: frame→world.
/// Only rows whose body is inside the frame region (Frame.Inside, by distance from P) are touched.</para>
/// </summary>
internal static unsafe class FrameStubs
{
    private enum Kind { None, Transform, Body, Gravity, Soft }

    private sealed class Slot
    {
        internal int Index;
        internal Kind Kind;
        internal InvocationStubData.DataAccess Access;
    }

    private sealed class Desc
    {
        internal string Name = "";
        internal Slot[] Slots = Array.Empty<Slot>();
        internal int TransformSlot = -1;
        internal InvocationStubData.DataAccess TransformAccess;
        internal long Rows;
        internal long Triggered;
    }

    private sealed class State
    {
        internal Desc Desc = null!;
        internal IntPtr[] Bases = Array.Empty<IntPtr>();
        internal int[] Strides = Array.Empty<int>();
        internal int Loops;
        internal bool[] Inside = Array.Empty<bool>();
        internal Vector3D[] WorldPos = Array.Empty<Vector3D>();
        internal byte[]? Saved;   // ReadOnly originals, per slot per row
        internal int[] SavedOffset = Array.Empty<int>();   // per slot index, -1 = not saved
    }

    private static readonly Dictionary<MethodBase, Desc> Descs = new();

    /// <summary>Declaring types whose pose-touching stubs belong to the physics boundary (cluster logic in
    /// HavokLargeWorldManager is deliberately excluded: it works on world-space AABBs).</summary>
    private static readonly string[] BoundaryTypes =
    {
        "Keen.VRage.Physics.Havok.HavokSessionComponent",
        "Keen.VRage.Physics.Havok.MotionSharing.MultiEntitySharedMotionSessionComponent",
    };

    internal static void Install(Harmony harmony)
    {
        Assembly physics = typeof(Keen.VRage.Physics.Havok.HavokSessionComponent).Assembly;
        var pre = new HarmonyMethod(typeof(FrameStubs), nameof(Prefix));
        var post = new HarmonyMethod(typeof(FrameStubs), nameof(Postfix));
        int patched = 0;
        foreach (Type t in physics.GetTypes())
        {
            if (Array.IndexOf(BoundaryTypes, t.FullName) < 0 && !(t.DeclaringType != null && Array.IndexOf(BoundaryTypes, t.DeclaringType.FullName) >= 0))
                continue;
            foreach (MethodInfo m in t.GetMethods(BindingFlags.Static | BindingFlags.NonPublic | BindingFlags.Public | BindingFlags.DeclaredOnly))
            {
                if (m.GetCustomAttribute<InvocationStub>() == null) continue;
                var data = m.GetCustomAttributes<InvocationStubData>();
                var slots = new List<Slot>();
                int index = 0, transformSlot = -1;
                var transformAccess = InvocationStubData.DataAccess.ReadOnly;
                foreach (InvocationStubData d in data)
                {
                    Kind k = d.Type == typeof(WorldTransform) ? Kind.Transform
                        : d.Type == typeof(RigidBodyData) ? Kind.Body
                        : d.Type == typeof(CustomGravity) ? Kind.Gravity
                        : d.Type == typeof(SoftTransform) ? Kind.Soft
                        : Kind.None;
                    bool rowAccess = d.AccessType == InvocationStubData.DataAccess.ReadOnly
                                  || d.AccessType == InvocationStubData.DataAccess.Writable
                                  || d.AccessType == InvocationStubData.DataAccess.Write;
                    if (k != Kind.None && rowAccess && !d.Optional)
                    {
                        slots.Add(new Slot { Index = index, Kind = k, Access = d.AccessType });
                        if (k == Kind.Transform) { transformSlot = index; transformAccess = d.AccessType; }
                    }
                    index++;
                }
                if (slots.Count == 0) continue;
                Descs[m] = new Desc { Name = t.Name + "." + m.Name.Replace("_InvocationStub", ""), Slots = slots.ToArray(), TransformSlot = transformSlot, TransformAccess = transformAccess };
                try
                {
                    harmony.Patch(m, pre, post);
                    patched++;
                    string list = string.Join(",", Array.ConvertAll(Descs[m].Slots, s => $"{s.Kind}:{s.Access}"));
                    ProbeLog.Write($"frame: wrapped {Descs[m].Name} [{list}]");
                }
                catch (Exception e) { ProbeLog.Write($"frame: wrap {m.Name} failed: {e.Message}"); }
            }
        }
        ProbeLog.Write($"frame: {patched} physics stubs wrapped");
    }

    private static int SizeOf(Kind k) => k switch
    {
        Kind.Transform => sizeof(WorldTransform),
        Kind.Body => sizeof(RigidBodyData),
        Kind.Gravity => sizeof(CustomGravity),
        Kind.Soft => sizeof(SoftTransform),
        _ => 0,
    };

    private static void Prefix(MethodBase __originalMethod, byte** __0, int __1, Keen.VRage.DCS.Scenes.Scene __2, out object? __state)
    {
        __state = null;
        if (!Frame.Active || __1 <= 0 || !Descs.TryGetValue(__originalMethod, out Desc? desc)) return;
        // Audit: how many rows an EVENT job really processes (trigger bits at dataPointers[-2], one bit per row - see the
        // loop in HavokSessionComponent.cs:2044-2061). Only the two collectors that turn ECS changes into Havok writes.
        if (desc.Name.EndsWith("CollectDataForStaticBodies") || desc.Name.EndsWith("CollectDataForDynamicBodies"))
        {
            ulong* bits = (ulong*)__0[-2];
            for (int w = 0; w < (__1 + 63) / 64; w++) desc.Triggered += System.Numerics.BitOperations.PopCount(bits[w] & (w == (__1 - 1) / 64 && __1 % 64 != 0 ? (1UL << (__1 % 64)) - 1 : ulong.MaxValue));
        }
        var st = new State { Desc = desc, Loops = __1 };
        int maxSlot = 0;
        foreach (Slot s in desc.Slots) maxSlot = Math.Max(maxSlot, s.Index);
        if (desc.TransformSlot > maxSlot) maxSlot = desc.TransformSlot;
        st.Bases = new IntPtr[maxSlot + 1];
        st.Strides = new int[maxSlot + 1];
        int* strides = (int*)__0[-1];
        for (int i = 0; i <= maxSlot; i++) { st.Bases[i] = (IntPtr)__0[i]; st.Strides[i] = strides[i]; }
        st.Inside = new bool[__1];
        st.WorldPos = new Vector3D[__1];

        // Which rows are in the frame region, and their WORLD position (for omega x r). For Write-only transform slots the
        // data is not valid yet; membership is decided again in the postfix from the frame position (same distance).
        bool anyInside = false;
        if (desc.TransformSlot >= 0)
        {
            for (int row = 0; row < __1; row++)
            {
                WorldTransform* wt = (WorldTransform*)(st.Bases[desc.TransformSlot] + row * st.Strides[desc.TransformSlot]);
                st.WorldPos[row] = wt->Position;
                st.Inside[row] = Frame.Inside(wt->Position);
                anyInside |= st.Inside[row];
            }
            // Write-only transform (read-back stubs): the data is about to be overwritten by Havok, the postfix decides per
            // row from the frame position. Never skip those here.
            if (desc.TransformAccess == InvocationStubData.DataAccess.Write) anyInside = true;
        }
        else
        {
            // No transform in this job (gravity-only stubs): find each row's position through its DEntity (slot 0).
            // EntityData<T>(Scene) random access: VRage.DCS\...\Accessors\EntityData.cs:25, :87 TryReadFrom.
            var transforms = new Keen.VRage.DCS.Accessors.EntityData<WorldTransform>(__2);
            for (int row = 0; row < __1; row++)
            {
                DEntity e = *(DEntity*)(st.Bases[0] + row * st.Strides[0]);
                if (transforms.TryReadFrom(e, out WorldTransform wt))
                {
                    st.WorldPos[row] = wt.Position;
                    st.Inside[row] = Frame.Inside(wt.Position);
                    anyInside |= st.Inside[row];
                }
            }
        }
        if (!anyInside) return;

        // Fixed per-slot offsets into the saved buffer (declaration order), so prefix and postfix agree whatever order
        // they visit slots in.
        int savedBytes = 0;
        st.SavedOffset = new int[maxSlot + 1];
        for (int i = 0; i <= maxSlot; i++) st.SavedOffset[i] = -1;
        foreach (Slot s in desc.Slots)
            if (s.Access == InvocationStubData.DataAccess.ReadOnly) { st.SavedOffset[s.Index] = savedBytes; savedBytes += SizeOf(s.Kind) * __1; }
        if (savedBytes > 0) st.Saved = new byte[savedBytes];

        double theta = Frame.ThetaEcs;
        // Convert velocities first: they need the WORLD position, before the transform slot is converted in place.
        foreach (Slot s in Ordered(desc.Slots))
        {
            if (s.Access == InvocationStubData.DataAccess.Write) continue;
            int size = SizeOf(s.Kind);
            for (int row = 0; row < __1; row++)
            {
                byte* p = (byte*)(st.Bases[s.Index] + row * st.Strides[s.Index]);
                if (s.Access == InvocationStubData.DataAccess.ReadOnly)
                    fixed (byte* dst = &st.Saved![st.SavedOffset[s.Index] + row * size]) Buffer.MemoryCopy(p, dst, size, size);
                if (!st.Inside[row]) continue;
                switch (s.Kind)
                {
                    case Kind.Transform: Frame.ToFrame(ref *(WorldTransform*)p, theta); break;
                    case Kind.Soft: Frame.ToFrame(ref ((SoftTransform*)p)->Value, theta); break;
                    case Kind.Body:
                        if (desc.TransformSlot >= 0) Frame.VelToFrame(ref *(RigidBodyData*)p, st.WorldPos[row], theta);
                        break;
                    case Kind.Gravity: ((CustomGravity*)p)->Value = Frame.RotF(((CustomGravity*)p)->Value, -theta); break;
                }
            }
        }
        desc.Rows += __1;
        __state = st;
    }

    private static void Postfix(object? __state)
    {
        if (__state is not State st) return;
        Desc desc = st.Desc;
        double theta = Frame.ThetaForWorld;

        // Transforms first (world position is needed for the velocity term), then everything else.
        foreach (Slot s in Ordered(desc.Slots, transformsFirst: true))
        {
            int size = SizeOf(s.Kind);
            for (int row = 0; row < st.Loops; row++)
            {
                byte* p = (byte*)(st.Bases[s.Index] + row * st.Strides[s.Index]);
                if (s.Access == InvocationStubData.DataAccess.ReadOnly)
                {
                    fixed (byte* src = &st.Saved![st.SavedOffset[s.Index] + row * size]) Buffer.MemoryCopy(src, p, size, size);
                    continue;
                }
                bool inside = st.Inside[row];
                if (s.Access == InvocationStubData.DataAccess.Write && s.Kind == Kind.Transform)
                    inside = Frame.Inside(((WorldTransform*)p)->Position);   // distance is frame-invariant
                if (!inside) continue;
                switch (s.Kind)
                {
                    case Kind.Transform:
                        Frame.ToWorld(ref *(WorldTransform*)p, theta);
                        st.WorldPos[row] = ((WorldTransform*)p)->Position;
                        st.Inside[row] = true;
                        break;
                    case Kind.Soft: Frame.ToWorld(ref ((SoftTransform*)p)->Value, theta); break;
                    case Kind.Body:
                        if (desc.TransformSlot >= 0) Frame.VelToWorld(ref *(RigidBodyData*)p, st.WorldPos[row], theta);
                        break;
                    case Kind.Gravity: ((CustomGravity*)p)->Value = Frame.RotF(((CustomGravity*)p)->Value, theta); break;
                }
            }
        }
    }

    /// <summary>Prefix: velocities/gravity before transforms (velocities need the WORLD position, captured before the transform
    /// slot is converted in place). Postfix: transforms first (the world position for omega x r comes from the converted
    /// transform). Saved originals use fixed per-slot offsets, so the order does not matter for restoring.</summary>
    private static IEnumerable<Slot> Ordered(Slot[] slots, bool transformsFirst = false)
    {
        foreach (Slot s in slots)
            if ((s.Kind == Kind.Transform || s.Kind == Kind.Soft) == transformsFirst) yield return s;
        foreach (Slot s in slots)
            if ((s.Kind == Kind.Transform || s.Kind == Kind.Soft) != transformsFirst) yield return s;
    }

    internal static string Stats()
    {
        var parts = new List<string>();
        foreach (Desc d in Descs.Values) if (d.Rows > 0) parts.Add(d.Triggered > 0 ? $"{d.Name}={d.Rows}(triggered {d.Triggered})" : $"{d.Name}={d.Rows}");
        foreach (Desc d in Descs.Values) { d.Rows = 0; d.Triggered = 0; }
        return string.Join(" ", parts);
    }
}
