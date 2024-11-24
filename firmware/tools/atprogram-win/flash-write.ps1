
&"C:\Program Files (x86)\Atmel\Studio\7.0\atbackend\atprogram.exe" `
    -t avrispmk2 `
    -i isp `
    -d attiny13a `
    program `
    -c `
    --flash `
    -f build/battery_checker.elf `
    --format elf `
    --verify
