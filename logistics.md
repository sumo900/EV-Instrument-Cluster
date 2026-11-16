## Design decisions so far

Two safety layers: the ESP32 does monitoring and the primary cutoff, and the protection board is the independent hardware backstop.
The software cutoff sits above the hardware cutoff: warn at about 3.5 V, cut off at about 3.3 V, with the hardware over-discharge limit at about 2.4 V.
The cutoff is a latched fault that needs a manual reset, so the load’s voltage sag doesn’t make it chatter.
The alarm should be local (a buzzer on a GPIO), so it works even if Wi-Fi drops. The dashboard only displays.
The XIAO stays on the pack and keeps drawing current after cutoff, so after a critical fault it should drop Wi-Fi or enter deep sleep.
Parallel cells must be matched within about 0.05 V before connection.
Charging the pack is slow (about 36 hours from empty on 3P at 100 mA). The plan is to demo only the discharge window, so a full charge isn’t needed for each rehearsal.