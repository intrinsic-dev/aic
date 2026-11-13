# AIC Docs (Sphinx) Docker image

Build and serve the Sphinx docs with live reload.

## Launching the docs

### Get media assets from media branch

Images and other assets live on a different branch  `media`, fetch them **before** starting the docs server so Sphinx can render them.

From the **repository root**:

```bash
# 1. Fetch and overlay assets from the media branch into docs
./docker/aic_docs/fetch_docs_assets.sh media

# 2. Start the docs server
cd docker && docker compose up docs
```

Then open **http://localhost:8000**.

> [!NOTE] 
> Re-run the script whenever you want to refresh assets from that branch.


---

## `fetch_docs_assets.sh` script

Run from the **repository root**. It fetches the given branch from `origin` and overlays its content into the docs tree so the Docker build can render assets.

**Usage:**

```bash
./docker/aic_docs/fetch_docs_assets.sh [branch]
```

- **`branch`** – Branch name (default: `media`). Must exist as `origin/<branch>` (run `git fetch origin` if needed).

**What it does:**

- Fetches `origin/<branch>`.
- Extracts the **root** of that branch into `docs/sphinx/source/_static/assets/` (overwriting that directory).  
  So the branch can have files at its root or in subdirs; they all end up under `_static/assets/` where the docs reference them.

**Example:**

```bash
./docker/aic_docs/fetch_docs_assets.sh media
# Then: cd docker && docker compose up docs
```