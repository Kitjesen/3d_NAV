# LingTu Documentation

This docs tree is intentionally narrowed to daily use, deployment, and operations.

## Start here

- [`QUICKSTART.md`](./QUICKSTART.md): fastest path to run LingTu.
- [`01-getting-started/`](./01-getting-started/): setup, build, and first-run guides.
- [`04-deployment/`](./04-deployment/): service layout, OTA, and deployment procedures.
- [`07-testing/`](./07-testing/): field and acceptance test checklists/scripts.
- [`03-development/`](./03-development/): troubleshooting and parameter-tuning workflow.
- [`TUNING.md`](./TUNING.md): runtime tuning reference.
- [`REPO_LAYOUT.md`](./REPO_LAYOUT.md): repository structure.

## Entry point

Use the unified CLI:

```bash
python lingtu.py [profile]
```

Common profiles:

```bash
python lingtu.py map
python lingtu.py nav
python lingtu.py explore
```

## Archived docs

Older design notes, papers, roadmap content, and deep internal references were moved to:

```text
docs/archive/
```
