#!/usr/bin/env bash
# Create/update only the explicitly selected private organization repository.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
REPO=ZenithFuturesLab/zav-simulator
command -v gh >/dev/null || { echo 'Install GitHub CLI: https://cli.github.com/'; exit 1; }
gh auth status --hostname github.com
./scripts/verify.sh
python3 scripts/audit_package.py
if [[ -z "$(git config user.name || true)" ]]; then
    git config user.name "$(gh api user --jq .login)"
fi
if [[ -z "$(git config user.email || true)" ]]; then
    identity=$(gh api user --jq '(.id|tostring)+"+"+.login+"@users.noreply.github.com"')
    git config user.email "$identity"
fi
git add -- .
if ! git diff --cached --quiet; then
    git commit -m 'Package ZAV simulator with locked dependencies and portable setup'
fi
if gh repo view "$REPO" --json isPrivate >/dev/null 2>&1; then
    [[ "$(gh repo view "$REPO" --json isPrivate --jq .isPrivate)" == true ]] || { echo 'Refusing to upload to a public repository.' >&2; exit 1; }
    if git remote get-url origin >/dev/null 2>&1; then
        [[ "$(git remote get-url origin)" == "https://github.com/$REPO.git" ]] || { echo 'Unexpected origin; refusing to replace it.' >&2; exit 1; }
    else
        git remote add origin "https://github.com/$REPO.git"
    fi
else
    gh repo create "$REPO" --private --description 'ZAV GPS-free Isaac Sim 5.1.0 / PX4 / FAST-LIVO2 simulator' --source . --remote origin
fi
branch=$(git symbolic-ref --quiet --short HEAD) || { echo 'Checkout a branch before publishing.' >&2; exit 1; }
git push --set-upstream origin "$branch"
gh repo view "$REPO" --json url,isPrivate,defaultBranchRef
