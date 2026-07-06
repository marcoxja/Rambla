#!/usr/bin/env bash
# Sync an allowlisted snapshot of rambla-private to the public marcoxja/Rambla repo (dev branch).
#
# Usage:
#   scripts/sync-to-public.sh              # dry run: copy + audit + scan + summary, no git writes
#   scripts/sync-to-public.sh --push       # after a clean dry run, also commit + push (asks for typed confirmation)
#
# Only paths in ALLOWLIST below are ever copied. Everything else (including
# .claude/, .git/, .github/, and anything not explicitly listed) is left
# behind by construction, not by exclusion.

set -euo pipefail

PRIVATE_REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PUBLIC_REPO="marcoxja/Rambla"
PUBLIC_BRANCH="dev"

ALLOWLIST=(
  "README.md"
  "DESIGN_SPEC.md"
  "PROJECT_VISION.md"
  "LICENSE"
  ".gitignore"
  "docs"
  "assets"
  "src"
  "scripts"
  "shared"
)

DO_PUSH=false
for arg in "$@"; do
  case "$arg" in
    --push) DO_PUSH=true ;;
    *) echo "Unknown argument: $arg" >&2; exit 1 ;;
  esac
done

log()  { printf '\n== %s ==\n' "$1"; }
fail() { printf '\nBLOCKED: %s\n' "$1" >&2; exit 1; }

command -v gh >/dev/null 2>&1 || fail "gh CLI not found on PATH."
command -v rsync >/dev/null 2>&1 || fail "rsync not found on PATH."

log "Checking gh auth and public repo reachability"
gh auth status >/dev/null 2>&1 || fail "gh is not authenticated. Run: gh auth login"
gh repo view "$PUBLIC_REPO" >/dev/null 2>&1 || fail "Could not reach $PUBLIC_REPO. Does it exist? (gh repo view $PUBLIC_REPO)"

SCRATCH_DIR="$(mktemp -d "${TMPDIR:-/tmp}/rambla-sync-XXXXXX")"
trap 'rm -rf "$SCRATCH_DIR"' EXIT

log "Cloning $PUBLIC_REPO ($PUBLIC_BRANCH) into scratch dir"
gh repo clone "$PUBLIC_REPO" "$SCRATCH_DIR/public" -- --quiet
cd "$SCRATCH_DIR/public"
git checkout -B "$PUBLIC_BRANCH" "origin/$PUBLIC_BRANCH" --quiet

log "Copying allowlisted paths"
for entry in "${ALLOWLIST[@]}"; do
  src="$PRIVATE_REPO_ROOT/$entry"
  dst="$SCRATCH_DIR/public/$entry"
  if [ ! -e "$src" ]; then
    echo "  (skip) $entry does not exist in private repo"
    continue
  fi
  if [ -d "$src" ]; then
    mkdir -p "$dst"
    rsync -a --delete "$src/" "$dst/"
    echo "  copied dir  $entry/"
  else
    rsync -a "$src" "$dst"
    echo "  copied file $entry"
  fi
done

# --- Section 3: filename audit (defense in depth) ---
log "Auditing filenames for internal/Claude/secret patterns"

HARD_BLOCK_FOUND=false
SOFT_WARN_FOUND=false

while IFS= read -r -d '' f; do
  rel="${f#"$SCRATCH_DIR"/public/}"
  lower_rel="$(printf '%s' "$rel" | tr '[:upper:]' '[:lower:]')"
  base="$(basename "$f")"
  lower_base="$(printf '%s' "$base" | tr '[:upper:]' '[:lower:]')"

  # Hard blocks
  case "$lower_rel" in
    *.claude*|*claude.md) echo "  HARD BLOCK: $rel (claude-related path)"; HARD_BLOCK_FOUND=true ;;
  esac
  case "$lower_base" in
    *.pem|*.key|id_rsa*|id_ed25519*|*.p12|*.pfx|credentials*.json) \
      echo "  HARD BLOCK: $rel (key/credential file)"; HARD_BLOCK_FOUND=true ;;
    .env|.env.*) \
      if [ "$lower_base" != ".env.example" ]; then
        echo "  HARD BLOCK: $rel (env file)"; HARD_BLOCK_FOUND=true
      fi ;;
  esac
  # Any unexpected dotfile/dotdir other than .gitignore
  case "$base" in
    .gitignore) : ;;
    .*) echo "  HARD BLOCK: $rel (unexpected dotfile/dotdir)"; HARD_BLOCK_FOUND=true ;;
  esac

  # Soft warnings
  case "$lower_base" in
    *token*|*secret*) echo "  SOFT WARNING: $rel (filename suggests token/secret — verify manually)"; SOFT_WARN_FOUND=true ;;
  esac
done < <(find "$SCRATCH_DIR/public" -type f -not -path '*/.git/*' -not -path '*/.github/*' -print0)

if [ "$HARD_BLOCK_FOUND" = false ] && [ "$SOFT_WARN_FOUND" = false ]; then
  echo "  clean — no filename issues found"
fi

# --- Section 4: secret content scanning ---
log "Scanning file contents for secrets"

SECRET_FOUND=false

if command -v gitleaks >/dev/null 2>&1; then
  echo "  running gitleaks..."
  if ! gitleaks detect --source "$SCRATCH_DIR/public" --no-git -v; then
    SECRET_FOUND=true
  fi
else
  echo "  gitleaks not installed — skipping (recommended: brew install gitleaks)"
fi

echo "  running fallback regex scan..."
# Patterns kept deliberately simple; report file+line+pattern name only, never the match text.
# Parallel indexed arrays (not associative) for bash 3.2 compatibility (macOS default /bin/bash).
PATTERN_NAMES=(
  "private key header"
  "OpenAI-style key"
  "Anthropic-style key"
  "AWS access key id"
  "GitHub token"
  "generic api/secret key assignment"
  "generic password assignment"
)
PATTERN_REGEXES=(
  '-----BEGIN (RSA|OPENSSH|EC|DSA|PRIVATE) KEY-----'
  'sk-[A-Za-z0-9]{20,}'
  'sk-ant-[A-Za-z0-9_-]{20,}'
  'AKIA[0-9A-Z]{16}'
  '(ghp_[A-Za-z0-9]{30,}|github_pat_[A-Za-z0-9_]{30,})'
  '(api|secret)[_-]?key[[:space:]]*[:=][[:space:]]*["'"'"'][A-Za-z0-9_-]{16,}["'"'"']'
  'password[[:space:]]*[:=][[:space:]]*["'"'"'].+["'"'"']'
)

while IFS= read -r -d '' f; do
  rel="${f#"$SCRATCH_DIR"/public/}"
  i=0
  while [ "$i" -lt "${#PATTERN_NAMES[@]}" ]; do
    name="${PATTERN_NAMES[$i]}"
    regex="${PATTERN_REGEXES[$i]}"
    i=$((i + 1))
    if grep -InE "$regex" "$f" >/tmp/rambla-sync-hit.$$ 2>/dev/null; then
      while IFS=: read -r lineno _rest; do
        echo "  SECRET FOUND: $rel:$lineno (pattern: $name)"
      done < /tmp/rambla-sync-hit.$$
      SECRET_FOUND=true
    fi
    rm -f /tmp/rambla-sync-hit.$$
  done
done < <(find "$SCRATCH_DIR/public" -type f -not -path '*/.git/*' -not -path '*/.github/*' -print0)

if [ "$SECRET_FOUND" = false ]; then
  echo "  clean — no secret patterns matched"
fi

# --- Summary ---
log "Summary of changes vs. origin/$PUBLIC_BRANCH"
git -C "$SCRATCH_DIR/public" add -A
git -C "$SCRATCH_DIR/public" status --short
echo
git -C "$SCRATCH_DIR/public" diff --cached --stat || true

if [ "$HARD_BLOCK_FOUND" = true ] || [ "$SECRET_FOUND" = true ]; then
  fail "Hard-block or secret findings above must be resolved before this can be pushed."
fi

if [ "$SOFT_WARN_FOUND" = true ]; then
  echo
  echo "Soft warnings were found above (see 'SOFT WARNING' lines)."
  read -r -p "Type CONTINUE to proceed past these warnings, anything else to stop: " warn_confirm
  [ "$warn_confirm" = "CONTINUE" ] || fail "Stopped at soft-warning confirmation."
fi

if [ "$DO_PUSH" = false ]; then
  echo
  echo "Dry run complete. No commit or push was made. Re-run with --push to publish."
  exit 0
fi

log "Ready to push to $PUBLIC_REPO ($PUBLIC_BRANCH)"
echo "This will commit and push the changes shown above."
read -r -p "Type PUSH to confirm, anything else to abort: " push_confirm
[ "$push_confirm" = "PUSH" ] || fail "Push aborted by user (confirmation not entered)."

cd "$SCRATCH_DIR/public"
SHORT_SHA="$(git -C "$PRIVATE_REPO_ROOT" rev-parse --short HEAD)"
SYNC_DATE="$(date +%Y-%m-%d)"
git commit -m "Sync from rambla-private @ ${SHORT_SHA} (${SYNC_DATE})"
git push origin "$PUBLIC_BRANCH"

log "Done"
echo "Pushed to https://github.com/${PUBLIC_REPO}/tree/${PUBLIC_BRANCH}"
