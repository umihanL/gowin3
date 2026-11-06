#!/usr/bin/env bash
# Run locally on X5 (prefer the serial console for first Wi-Fi setup).
set -euo pipefail
action="${1:---status}"
interface="${2:-wlan0}"
case "$action" in
  --status|--connect) ;;
  *) echo 'Usage: bash tools/connect_phone_hotspot.sh [--status|--connect] [wlan0]' >&2; exit 2 ;;
esac
[[ "$interface" =~ ^[a-zA-Z0-9_.-]+$ ]] || { echo 'Invalid interface name.' >&2; exit 2; }
command -v nmcli >/dev/null || { echo 'NetworkManager/nmcli is required on X5.' >&2; exit 1; }
command -v ip >/dev/null || { echo 'iproute2 is required.' >&2; exit 1; }
if [ "$action" = --connect ]; then
  [ "$(id -u)" = 0 ] || { echo 'Connect with sudo; status does not require sudo.' >&2; exit 1; }
  [ -t 0 ] || { echo 'Connect requires an interactive terminal for the SSID/password.' >&2; exit 1; }
  nmcli -f DEVICE,TYPE,STATE,CONNECTION device status
  if [ "$(nmcli -g GENERAL.TYPE device show "$interface")" != wifi ]; then
    echo "Not a Wi-Fi interface: $interface" >&2; exit 1
  fi
  nmcli radio wifi on
  nmcli -f IN-USE,SSID,SIGNAL,SECURITY device wifi list ifname "$interface" --rescan yes
  read -r -p 'Phone hotspot SSID: ' hotspot_ssid
  [ -n "$hotspot_ssid" ] || { echo 'SSID is empty.' >&2; exit 2; }
  # Let NetworkManager ask for the secret; it is never placed in command history.
  nmcli --ask --wait 35 device wifi connect "$hotspot_ssid" ifname "$interface"
  connection_uuid="$(nmcli -g GENERAL.CON-UUID device show "$interface")"
  [[ "$connection_uuid" =~ ^[0-9a-fA-F-]{36}$ ]] || { echo 'Connected, but no valid connection UUID returned.' >&2; exit 1; }
  nmcli connection modify uuid "$connection_uuid" connection.autoconnect yes
fi
nmcli -f DEVICE,TYPE,STATE,CONNECTION device status
ip -brief -4 address show dev "$interface"
echo 'Use the wlan address above in the desktop console, with port 8090.'
echo 'Connect the PC to the same phone hotspot. The phone must allow clients to communicate.'
echo 'Ethernet and the existing 192.168.127.10 configuration were not changed.'
