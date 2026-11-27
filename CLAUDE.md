# Project rules

## Cite the source of every patch
All newly written code must include a comment, directly above it, naming the source it patches, hooks, reads from or is modelled on.

- Point at the decompiled source: file path under `D:\SE2MPNEW\Se2Decomp` plus class.method and a line number when known, e.g.
  `// Patches: Game2.Simulation/.../GravityGeneratorProcessorComponent.cs  GravityGeneratorProcessorComponent.UpdateOB (:142)`
- Use `// Patches:` for Harmony patches and job overrides, `// From:` for logic copied or adapted from game code, and `// Reads:` for game data/components consumed. Use `// Source:` for an external reference (paper, formula, doc URL).
- Applies to new Harmony patches, jobs, helpers that mirror game behaviour, and constants taken from game code.
- If no game source applies (pure plugin logic), say so: `// Source: none (plugin-only)`.
- When editing existing code that lacks a reference, add one for the part you touched.
