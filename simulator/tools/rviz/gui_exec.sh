#!/usr/bin/env bash
# Qt/GTK from the host must not load VS Code Snap's private glibc libraries.
unset GTK_PATH GTK_EXE_PREFIX GTK_DATA_PREFIX GIO_MODULE_DIR LD_PRELOAD
if [ -n "${LD_LIBRARY_PATH:-}" ]; then
    _zav_gui_libs=""
    IFS=: read -ra _zav_gui_entries <<< "$LD_LIBRARY_PATH"
    for entry in "${_zav_gui_entries[@]}"; do
        case "$entry" in /snap/*|'') continue ;; esac
        _zav_gui_libs="${_zav_gui_libs:+$_zav_gui_libs:}$entry"
    done
    export LD_LIBRARY_PATH="$_zav_gui_libs"
fi
exec "$@"
