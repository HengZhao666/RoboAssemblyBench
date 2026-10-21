# Fabrica canonical seven-bundle handoff

This package contains the seven canonical official-aligned Fabrica task bundles exported from PCNL-41 on 2026-07-14.

## Included assemblies

| Assembly | Parts | Bundle directory |
|---|---:|---|
| beam | 5 | `fabrica_official_beam_fixture_pickup_sdf512_v1` |
| car | 6 | `fabrica_official_car_fixture_pickup_sdf512_v1` |
| cooling_manifold | 7 | `fabrica_official_cooling_manifold_fixture_pickup_sdf512_v1` |
| duct | 8 | `fabrica_official_duct_fixture_pickup_sdf512_v1` |
| gamepad | 6 | `fabrica_official_gamepad_fixture_pickup_sdf512_v1` |
| plumbers_block | 5 | `fabrica_official_plumbers_block_fixture_pickup_sdf512_v1` |
| stool_circular | 9 | `fabrica_official_stool_circular_fixture_pickup_sdf512_v1` |

The archive contains 46 unique canonical part assets in total. The optional `plumbers_block` `margin000` and `margin005` fixture experiments are not included.

## Bundle contract

Keep each bundle directory intact. Its main entrypoint is `scene/scene.usda`, whose relative references resolve the bundled optical board, official fixture, pickup parts and assembled final-pose reference.

Each canonical bundle includes:

- `PACKAGE_MANIFEST.json` and `README.md`
- `scene/scene.usda` and `scene/scene_spec.json`
- raw-frame part USD assets in `assets/fabrica_parts/`
- official fixture, pickup JSON and optical board assets
- grasp, precedence, plan and tree metadata
- static and Isaac Sim runtime validation reports

All scene stages use `metersPerUnit = 1`. Source Fabrica meshes were converted from centimeters to meters while preserving the official raw-frame coordinates.

## Verification

From the extracted package root, run:

```bash
sha256sum -c SHA256SUMS
```

Then open any `task_bundles/<bundle>/scene/scene.usda` in Isaac Sim. Do not relocate files inside an individual bundle unless you also rewrite its relative USD references.

`handoff/handoff.json` records the generation state and known limitations. The original source directory on server 34 was not byte-for-byte compared because available SSH keys were rejected; the PCNL-41 bundle manifests preserve their official source provenance.
