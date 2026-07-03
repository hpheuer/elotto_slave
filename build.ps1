$env:IDF_PATH        = "C:\esp\v6.0.1\esp-idf"
$env:IDF_TOOLS_PATH  = "C:\Espressif\tools"
$env:ESP_IDF_VERSION = "6.0.1"
$env:PATH = "C:\Espressif\tools\python_env\idf6.0_py3.11_env\Scripts;" +
            "C:\Espressif\tools\riscv32-esp-elf\esp-15.2.0_20251204\riscv32-esp-elf\bin;" +
            "C:\Espressif\tools\cmake\3.30.2\bin;" +
            "C:\Espressif\tools\ninja\1.12.1;" +
            "C:\Espressif\tools\git\bin;" +
            $env:PATH

Set-Location $PSScriptRoot
& python "C:\esp\v6.0.1\esp-idf\tools\idf.py" $args
