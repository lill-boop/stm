$port = new-Object System.IO.Ports.SerialPort COM3, 9600, None, 8, One
$port.ReadTimeout = 2000
try {
    $port.Open()
    Write-Host "Port Opened. Reading continuously (Ctrl+C to stop)..."
    while ($true) { 
        if ($port.BytesToRead -gt 0) {
            $line = $port.ReadLine()
            Write-Host $line
        }
    }
}
catch {
    Write-Host "Error: $_"
}
finally {
    if ($port.IsOpen) {
        $port.Close()
    }
}
