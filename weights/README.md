# weights/

One directory per model. Add as many as you want — the dashboard picks up
every folder under here that contains a `.pt` or a Hailo `.hef`.

```
weights/
  <model_id>/                 # catalog id = this folder name
    model.yaml                # optional sidecar (label, task, track_label, …)
    best.pt                   # Ultralytics checkpoint (or model.pt)
    hailo/                    # preferred Hailo package (any *.hef)
    <stem>_hailo_model/       # Ultralytics/DFC export (also accepted)
    <stem>hailomodel/         # compact Hailo folder (also accepted)
    <stem>_ncnn_model/        # CPU fallback
    <stem>_openvino_model/
  _template/                  # copy this to start a new model
```

`config/model.yaml` `models:` entries overlay the same id (labels, `main: true`).
They are optional once a sidecar exists.

## `<model_id>` (folder name)

- Use `a-z`, `0-9`, `-`, `_` only. This string is the dashboard id.
- Examples: `detect`, `gate`, `lobster`, `gate-v2`.
- Do not start the name with `_` (`_template` is ignored).
- Do not use Ultralytics export names as the model id
  (`*_hailo_model`, `*hailomodel`, `*_ncnn_model`, `*_openvino_model`,
  `hailo`, `ncnn`).

Versions are extra folders, not nested files: `weights/detect-v2/`, not
`weights/detect/v2/`.

## Files inside the folder

| File / folder | Role |
|---------------|------|
| `model.yaml` | Sidecar. `label`, `task` (`detect` / `segment` / `auto`), `track_label` (class the tracker follows), `default_backend`, optional `weights: best.pt`. |
| `best.pt` or `model.pt` | Train/export source. If several `.pt` files exist, `model.pt` then `best.pt` then the first name wins. |
| `hailo/*.hef` | Preferred Hailo-8L package. Newest `.hef` in the folder is loaded. |
| `<stem>_hailo_model/` | What `scripts/export_model.py --format hailo` writes next to the `.pt`. |
| `<stem>hailomodel/` | Same as above without underscores (e.g. `besthailomodel/`). |
| `<stem>_ncnn_model/` | `export_model.py --format ncnn` |
| `<stem>_openvino_model/` | `export_model.py --format openvino` |

`<stem>` is the `.pt` filename without `.pt` (`best.pt` → `best_hailo_model/`).

## Add a model

```bash
cp -a weights/_template weights/lobster
# put lobster.pt or best.pt in that folder, edit model.yaml
python scripts/export_model.py --format hailo --weights weights/lobster/best.pt
```

The `/sub/` model row refreshes within a few seconds. Set `active_model:`
in `config/model.yaml` if it should load on boot.

Hailo-8L holds **one** HEF. A second Hailo model parks the first on NCNN/CPU.

## Legacy flat layout

These still work (and are symlinks into `detect/` / `gate/` on this Pi):

- `weights/best.pt`
- `weights/besthailomodel/`
- `weights/gatebest.pt`
- `weights/<stem>_hailo_model/`
