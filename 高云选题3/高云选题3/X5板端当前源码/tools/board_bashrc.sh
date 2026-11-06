# VS Code remote terminal initialization; no global shell configuration is changed.
if [ -f "$HOME/.bashrc" ]; then source "$HOME/.bashrc"; fi
source "$(dirname "${BASH_SOURCE[0]}")/../env.sh"
