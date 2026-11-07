# GitHub Upload Checklist

Use this checklist before uploading the repository.

## Required

- [ ] Choose a license and replace `LICENSE_PENDING.md` with a real `LICENSE` file.
- [ ] Update `CITATION.cff` authors and repository URL.
- [ ] Confirm the paper title and citation information.
- [ ] Confirm RflySim SDK files are not included.
- [ ] Confirm PX4 firmware, CopterSim binaries, and QGroundControl binaries are not included.
- [ ] Confirm logs, screenshots, videos, and CSV files are not included.
- [ ] Confirm local COM-port mappings are not included.
- [ ] Confirm private GPS coordinates are not included.

## Recommended GitHub Settings

- Repository name: `negotiated-time-bts-uav-hil`
- Visibility: public after license/citation confirmation
- Topics:
  - `uav`
  - `px4`
  - `rflysim`
  - `hardware-in-the-loop`
  - `multi-agent-systems`
  - `formation-control`
  - `bounded-time-control`
  - `time-consensus`

## Suggested First Commit

```bash
git init
git add .
git commit -m "Initial sanitized release"
git branch -M main
git remote add origin https://github.com/USERNAME/negotiated-time-bts-uav-hil.git
git push -u origin main
```

