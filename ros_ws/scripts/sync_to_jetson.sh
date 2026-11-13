#!/usr/bin/env bash
# Copy changed Dagozilla source files to an existing Jetson workspace.
# Default mode is a non-mutating preview; pass --apply to transfer files.
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/sync_to_jetson.sh [--apply] [--delete-removed] USER@JETSON:/remote/Dagozilla

Examples:
  # Preview exactly which source files would be sent.
  scripts/sync_to_jetson.sh rie@jetson:/home/rie/Dagozilla

  # Transfer only changed files; never deletes anything on Jetson.
  scripts/sync_to_jetson.sh --apply rie@jetson:/home/rie/Dagozilla

  # Also remove remote source files that were deleted locally.
  # Use only after reviewing the preview above.
  scripts/sync_to_jetson.sh --apply --delete-removed rie@jetson:/home/rie/Dagozilla
EOF
}

apply=false
delete_removed=false
destination=""

while (($#)); do
  case "$1" in
    --apply)
      apply=true
      ;;
    --delete-removed)
      delete_removed=true
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    -*)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
    *)
      if [[ -n "$destination" ]]; then
        echo "Only one Jetson destination is allowed." >&2
        usage >&2
        exit 2
      fi
      destination="$1"
      ;;
  esac
  shift
done

if [[ -z "$destination" || "$destination" != *:* ]]; then
  echo "Destination must use USER@JETSON:/remote/Dagozilla format." >&2
  usage >&2
  exit 2
fi

if "$delete_removed" && ! "$apply"; then
  echo "--delete-removed requires --apply." >&2
  exit 2
fi

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source_dir="$(cd -- "$script_dir/.." && pwd)"

rsync_options=(
  --archive
  --compress
  --checksum
  --no-times
  --omit-dir-times
  --no-owner
  --no-group
  --itemize-changes
  --human-readable
  --exclude=.git/
  --exclude=build/
  --exclude=install/
  --exclude=log/
  --exclude=__pycache__/
  --exclude=.pytest_cache/
  --exclude=.mypy_cache/
  --exclude='*.pyc'
  --exclude='*.swp'
  --exclude=.DS_Store
)

if ! "$apply"; then
  rsync_options+=(--dry-run)
  echo "Preview only — no files will be changed on Jetson."
elif "$delete_removed"; then
  rsync_options+=(--delete)
  echo "Applying changes and deleting remote files removed locally."
else
  echo "Applying changed/new files only; no remote files will be deleted."
fi

rsync "${rsync_options[@]}" "$source_dir/" "$destination/"

if "$apply"; then
  cat <<'EOF'

Source transfer complete. On Jetson, rebuild the affected ROS packages:
  cd ~/Dagozilla
  source /opt/ros/jazzy/setup.bash
  colcon build --symlink-install
  source install/setup.bash
EOF
fi
