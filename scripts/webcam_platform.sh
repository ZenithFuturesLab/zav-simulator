webcam_is_wsl() {
  [[ ${WSL_INTEROP:-} || $(uname -r) == *[Mm]icrosoft* ]]
}

webcam_ffmpeg_path() {
  local appdata
  appdata=$(powershell.exe -NoProfile -NonInteractive -Command '$env:LOCALAPPDATA' | tr -d '\r\n') || return
  [[ -n "$appdata" ]] || return 1
  printf '%s/ros-webcam/ffmpeg.exe\n' "$(wslpath -u "$appdata")"
}
