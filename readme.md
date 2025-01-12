# perkenalan

ini adalah repositori untuk firmware perangkat automatic gate nobo versi bluepill.

sebelumnya, automatic gate nobo menggunakan node mcu. namun karena dia berada didalam box panel, maka noise yang terjadi sangat parah. ditambah limit switch yang digunakan hanya ada 1 buah (2 buah, namun dicouple jadi 1) menyebabkan fungsi antara limit dekselerasi dengan limit berhenti tidak bisa dibedakan. maka dari itu update kali ini menggunakan bluepill dengan beberapa fitur tambahan

## ringkasan update hardware

- hardware berada diluar box panel dengan box sendiri dari 3d print agar terhindar dari noise yang tidak diinginkan
- limit dekselerasi dan limit berhenti dibaca sendiri-sendiri
- menggunakan topologi stack seperti yang ada pada PLC sehingga dapat mengurangi noise dari step-down
- modular module, sehingga lebih mudah untuk diupgrade di kemudian hari
- simple signal filter karena hanya menggunakan 1 ic 74hc245 dengan pullup, sehingga hanya mengandalkan koneksi ground untuk trigger
- menyisakan 1 module relay, dan 2 in-out 74hc245

## ringkasan update firmware

- menggunakan framework stm32cube sehingga kompatibilitas dengan bluepill sangat bagus
- pemrograman menggunakan bahasa c di cubeide. sebelumnya menggunakan c++ di platformio
- built-in rtc dan microsd untuk data logger yang bisa dianalisis dikemudian hari jika dibutuhkan
- interface menggunakan serial monitor untuk mengubah parameter dan debugging
