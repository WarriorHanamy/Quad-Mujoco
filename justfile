export UV_CACHE_DIR := ".uv_cache"

k:
        tmux kill-session
work:
        tmuxp load tmux_config/agents.yaml
view:
	uv run python view_entry.py

main:
	uv run python main_se3.py
