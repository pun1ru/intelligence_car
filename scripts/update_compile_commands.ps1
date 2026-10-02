param(
    [string]$TargetName = 'nor_sdram_zf_dtcm'
)

$ErrorActionPreference = 'Stop'

$workspaceRoot = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..'))
$projectPath = Join-Path $workspaceRoot 'embedded/SeekFree_RT1064_Opensource_Library/project/mdk/rt1064.uvprojx'
$projectDirectory = Split-Path -Parent $projectPath
$databasePath = Join-Path $workspaceRoot 'compile_commands.json'
$libraryPrefix = 'embedded/SeekFree_RT1064_Opensource_Library/libraries/myApplication/'

function ConvertTo-WorkspacePath([string]$absolutePath) {
    $fullPath = [IO.Path]::GetFullPath($absolutePath)
    $prefix = $workspaceRoot.TrimEnd('\') + '\'
    if (-not $fullPath.StartsWith($prefix, [StringComparison]::OrdinalIgnoreCase)) {
        throw "Path is outside the workspace: $fullPath"
    }
    return $fullPath.Substring($prefix.Length).Replace('\', '/')
}

[xml]$project = Get-Content -LiteralPath $projectPath -Raw -Encoding UTF8
$target = $project.SelectSingleNode("//Target[TargetName='$TargetName']")
if ($null -eq $target) {
    throw "Keil target not found: $TargetName"
}

$includeNode = $target.SelectSingleNode(".//IncludePath[contains(text(), 'myApplication')]")
$defineNode = $target.SelectSingleNode(".//Define[contains(text(), 'CPU_MIMXRT1064')]")
if (($null -eq $includeNode) -or ($null -eq $defineNode)) {
    throw 'Target include paths or preprocessor definitions were not found.'
}

$includePaths = [ordered]@{}
foreach ($path in $includeNode.InnerText.Split(';')) {
    if ([string]::IsNullOrWhiteSpace($path)) { continue }
    $relativePath = ConvertTo-WorkspacePath (Join-Path $projectDirectory $path.Trim())
    $includePaths[$relativePath] = $true
}

$previous = Get-Content -LiteralPath $databasePath -Raw -Encoding UTF8 | ConvertFrom-Json
foreach ($entry in $previous) {
    foreach ($argument in $entry.arguments) {
        if (-not $argument.StartsWith('-I')) { continue }
        $path = $argument.Substring(2).Replace('\', '/')
        if ($path.StartsWith($libraryPrefix, [StringComparison]::OrdinalIgnoreCase)) { continue }
        if (Test-Path -LiteralPath (Join-Path $workspaceRoot $path) -PathType Container) {
            $includePaths[$path] = $true
        }
    }
}

$sourceFiles = @{}
foreach ($entry in $previous) {
    $path = $entry.file.Replace('\', '/')
    $marker = $path.IndexOf('/embedded/', [StringComparison]::OrdinalIgnoreCase)
    if ($marker -ge 0) {
        $path = $path.Substring($marker + 1)
    }
    if ($path.StartsWith('embedded/', [StringComparison]::OrdinalIgnoreCase) -and
        (Test-Path -LiteralPath (Join-Path $workspaceRoot $path) -PathType Leaf)) {
        $sourceFiles[$path] = $true
    }
}

foreach ($file in $target.SelectNodes(".//File[FileType='1']/FilePath")) {
    $relativePath = ConvertTo-WorkspacePath (Join-Path $projectDirectory $file.InnerText)
    if (-not (Test-Path -LiteralPath (Join-Path $workspaceRoot $relativePath) -PathType Leaf)) {
        throw "Keil source file does not exist: $relativePath"
    }
    $sourceFiles[$relativePath] = $true
}

$arguments = @('clang', '--target=arm-arm-none-eabi', '-mcpu=cortex-m7',
               '-mthumb', '-std=c99', '-Wno-invalid-source-encoding')
foreach ($path in $includePaths.Keys) {
    $arguments += "-I$path"
}
foreach ($definition in $defineNode.InnerText.Split(',')) {
    $value = $definition.Trim() -replace '\s*=\s*', '='
    if ($value.Length -gt 0) {
        $arguments += "-D$value"
    }
}

$directory = $workspaceRoot.Replace('\', '/')
$database = foreach ($path in ($sourceFiles.Keys | Sort-Object)) {
    [pscustomobject][ordered]@{
        directory = $directory
        file = $path
        arguments = $arguments
    }
}

$json = ConvertTo-Json -InputObject @($database) -Depth 4
[IO.File]::WriteAllText($databasePath, $json + [Environment]::NewLine,
                        [Text.UTF8Encoding]::new($false))
Write-Output "Updated $databasePath with $($sourceFiles.Count) source files."
