$port = new-Object System.IO.Ports.SerialPort COM3, 9600, None, 8, One
$port.ReadTimeout = 2000
try {
    $port.Open()
    Write-Host "Port Opened. Waiting for data..."
    for ($i = 0; $i -lt 10; $i++) { 
        $line = $port.ReadLine()
        Write-Host $line
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
