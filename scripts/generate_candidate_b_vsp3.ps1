#Requires -Version 5.1

<#
.SYNOPSIS
Build and validate the real Douglas Dart Candidate B OpenVSP model.

.DESCRIPTION
Locates an OpenVSP 3.51.2 Windows distribution with Python 3.11 bindings,
installs the repository package into its existing virtual environment, builds
Candidate B, and reads the generated VSP3 file back through the real OpenVSP API.

.PARAMETER OpenVspRoot
Optional path to vsp.exe or to a directory containing an extracted OpenVSP
3.51.2 distribution. When omitted, common Windows locations are searched.
#>
[CmdletBinding()]
param(
    [Parameter(Mandatory = $false)]
    [string]$OpenVspRoot
)

$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

$ExpectedOpenVspVersion = "3.51.2"
$ExpectedPythonVersion = "3.11"
$VersionMarker = "__DOUGLAS_DART_OPENVSP_VERSION__="
$GeometryMarker = "__DOUGLAS_DART_OPENVSP_GEOMETRY__="

function Invoke-CheckedNativeCommand {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Executable,

        [Parameter(Mandatory = $true)]
        [string[]]$Arguments,

        [Parameter(Mandatory = $false)]
        [switch]$CaptureOutput
    )

    $global:LASTEXITCODE = 0
    if ($CaptureOutput) {
        $commandOutput = @(& $Executable @Arguments 2>&1)
        $exitCode = $LASTEXITCODE
        if ($exitCode -ne 0) {
            $details = ($commandOutput | ForEach-Object { $_.ToString() }) -join [Environment]::NewLine
            throw "Command failed with exit code ${exitCode}: $Executable $($Arguments -join ' ')`n$details"
        }
        return $commandOutput
    }

    & $Executable @Arguments
    $exitCode = $LASTEXITCODE
    if ($exitCode -ne 0) {
        throw "Command failed with exit code ${exitCode}: $Executable $($Arguments -join ' ')"
    }
}

function Get-UniqueExistingDirectories {
    param(
        [Parameter(Mandatory = $true)]
        [object[]]$Paths
    )

    $seen = @{}
    $directories = @()
    foreach ($pathValue in $Paths) {
        if ([string]::IsNullOrWhiteSpace([string]$pathValue)) {
            continue
        }

        $candidate = [Environment]::ExpandEnvironmentVariables([string]$pathValue)
        if (-not (Test-Path -LiteralPath $candidate -PathType Container)) {
            continue
        }

        $fullPath = (Get-Item -LiteralPath $candidate).FullName
        $key = $fullPath.ToUpperInvariant()
        if (-not $seen.ContainsKey($key)) {
            $seen[$key] = $true
            $directories += $fullPath
        }
    }
    return $directories
}

function Find-VspExecutable {
    param(
        [Parameter(Mandatory = $true)]
        [string]$RepositoryRoot,

        [Parameter(Mandatory = $false)]
        [string]$RequestedRoot
    )

    if (-not [string]::IsNullOrWhiteSpace($RequestedRoot)) {
        $expandedRoot = [Environment]::ExpandEnvironmentVariables($RequestedRoot)
        if (-not (Test-Path -LiteralPath $expandedRoot)) {
            throw "The -OpenVspRoot path does not exist: $expandedRoot"
        }

        $requestedItem = Get-Item -LiteralPath $expandedRoot
        if (-not $requestedItem.PSIsContainer) {
            if ($requestedItem.Name -ine "vsp.exe") {
                throw "-OpenVspRoot must identify vsp.exe or a directory containing OpenVSP: $expandedRoot"
            }
            return $requestedItem
        }

        $requestedMatches = @(Get-ChildItem -LiteralPath $requestedItem.FullName -File -Filter "vsp.exe" -Recurse -ErrorAction SilentlyContinue)
        if ($requestedMatches.Count -eq 0) {
            throw "vsp.exe was not found under -OpenVspRoot: $($requestedItem.FullName)"
        }
        return ($requestedMatches | Sort-Object FullName | Select-Object -First 1)
    }

    $userProfile = [Environment]::GetFolderPath("UserProfile")
    $programFiles = [Environment]::GetEnvironmentVariable("ProgramFiles")
    $programFilesX86 = [Environment]::GetEnvironmentVariable("ProgramFiles(x86)")
    $localAppData = [Environment]::GetFolderPath("LocalApplicationData")

    # Search the repository first, followed by normal download/install locations.
    # The user-profile root is last because it can be considerably larger.
    $searchRoots = Get-UniqueExistingDirectories -Paths @(
        $RepositoryRoot,
        (Join-Path $RepositoryRoot "openvsp"),
        (Join-Path $RepositoryRoot "tools"),
        (Join-Path $userProfile "Downloads"),
        $programFiles,
        $programFilesX86,
        (Join-Path $localAppData "Programs"),
        (Join-Path $userProfile "OpenVSP"),
        $userProfile
    )

    $fallbackMatches = @()
    $versionPattern = [Regex]::Escape($ExpectedOpenVspVersion)
    foreach ($searchRoot in $searchRoots) {
        Write-Host "Searching for OpenVSP under: $searchRoot"
        $matches = @(Get-ChildItem -LiteralPath $searchRoot -File -Filter "vsp.exe" -Recurse -ErrorAction SilentlyContinue)
        foreach ($match in ($matches | Sort-Object FullName)) {
            if ($match.FullName -match $versionPattern) {
                return $match
            }
            $fallbackMatches += $match
        }
    }

    # A custom extraction folder may omit the version from its name. Keep one
    # such candidate and let the real API version check below make the decision.
    if ($fallbackMatches.Count -gt 0) {
        return ($fallbackMatches | Sort-Object FullName | Select-Object -First 1)
    }

    throw "OpenVSP $ExpectedOpenVspVersion was not found. Install/extract the Python $ExpectedPythonVersion Windows bundle or pass -OpenVspRoot."
}

function Get-OpenVspPythonPaths {
    param(
        [Parameter(Mandatory = $true)]
        [string]$VspExecutableDirectory
    )

    $pythonBundleRoot = Join-Path $VspExecutableDirectory "python"
    if (-not (Test-Path -LiteralPath $pythonBundleRoot -PathType Container)) {
        $pythonCandidates = @(Get-ChildItem -LiteralPath $VspExecutableDirectory -Directory -Filter "python" -Recurse -ErrorAction SilentlyContinue)
        if ($pythonCandidates.Count -eq 0) {
            throw "The OpenVSP Python bundle was not found below: $VspExecutableDirectory"
        }
        $pythonBundleRoot = ($pythonCandidates | Sort-Object FullName | Select-Object -First 1).FullName
    }

    # OpenVSP ships several sibling modules. Include the bundle root and all
    # package/binary subdirectories so Python and the Windows DLL loader can find
    # openvsp, degen_geom, utilities, configuration helpers, and compiled .pyd files.
    $pythonPaths = @($pythonBundleRoot)
    $pythonPaths += @(Get-ChildItem -LiteralPath $pythonBundleRoot -Directory -Recurse -ErrorAction SilentlyContinue | ForEach-Object { $_.FullName })
    return (Get-UniqueExistingDirectories -Paths $pythonPaths)
}

try {
    # This script lives in <repository>\scripts, so its parent is the repository root.
    $repositoryRoot = (Get-Item -LiteralPath (Join-Path $PSScriptRoot "..")).FullName
    $pythonExecutable = Join-Path $repositoryRoot ".venv\Scripts\python.exe"
    $configurationRelativePath = "configs/shared_nozzle_candidate_b.yaml"
    $outputRelativePath = "openvsp/generated/shared_nozzle_candidate_b.vsp3"
    $configurationPath = Join-Path $repositoryRoot $configurationRelativePath
    $outputPath = Join-Path $repositoryRoot $outputRelativePath

    if (-not (Test-Path -LiteralPath $pythonExecutable -PathType Leaf)) {
        throw "Repository virtual-environment Python was not found: $pythonExecutable"
    }
    if (-not (Test-Path -LiteralPath $configurationPath -PathType Leaf)) {
        throw "Candidate B configuration was not found: $configurationPath"
    }

    $pythonVersionScript = "import sys; actual=str(sys.version_info.major) + '.' + str(sys.version_info.minor); print(actual); raise SystemExit(0 if actual == '3.11' else 1)"
    $pythonVersionOutput = @(Invoke-CheckedNativeCommand -Executable $pythonExecutable -Arguments @("-c", $pythonVersionScript) -CaptureOutput)
    $pythonVersion = ($pythonVersionOutput | ForEach-Object { $_.ToString().Trim() } | Where-Object { $_ -ne "" } | Select-Object -Last 1)
    if ($pythonVersion -ne $ExpectedPythonVersion) {
        throw "The repository virtual environment must use Python $ExpectedPythonVersion; found $pythonVersion."
    }

    $vspExecutable = Find-VspExecutable -RepositoryRoot $repositoryRoot -RequestedRoot $OpenVspRoot
    $vspExecutableDirectory = $vspExecutable.Directory.FullName
    $openVspPythonPaths = Get-OpenVspPythonPaths -VspExecutableDirectory $vspExecutableDirectory

    $environmentPaths = @($vspExecutableDirectory) + $openVspPythonPaths
    $pathPrefix = $environmentPaths -join ";"
    $env:PATH = "$pathPrefix;$env:PATH"
    if ([string]::IsNullOrWhiteSpace($env:PYTHONPATH)) {
        $env:PYTHONPATH = $openVspPythonPaths -join ";"
    }
    else {
        $env:PYTHONPATH = "$($openVspPythonPaths -join ';');$env:PYTHONPATH"
    }

    Write-Host "Using repository root: $repositoryRoot"
    Write-Host "Using Python: $pythonExecutable"
    Write-Host "Using OpenVSP executable directory: $vspExecutableDirectory"

    # Import both official modules and reject namespace-only, mocked, incomplete,
    # or version-mismatched APIs before installing/building the project.
    $verifyApiScript = @'
import sys
from pathlib import Path

import degen_geom
import openvsp

expected = "3.51.2"
distribution_root = Path(sys.argv[1]).resolve()
for module in (degen_geom, openvsp):
    module_file = Path(module.__file__).resolve()
    if distribution_root != module_file and distribution_root not in module_file.parents:
        raise RuntimeError(
            f"{module.__name__} was imported from outside the selected OpenVSP distribution: "
            f"{module_file}"
        )
if not hasattr(openvsp, "AddGeom") or not callable(openvsp.AddGeom):
    raise RuntimeError("openvsp.AddGeom is missing or is not callable")
actual = str(openvsp.GetVSPVersion()).replace("OpenVSP", "").strip()
if actual != expected:
    raise RuntimeError(f"OpenVSP API version mismatch: expected {expected}, got {actual}")
print("__DOUGLAS_DART_OPENVSP_VERSION__=" + actual)
'@
    $verifyOutput = @(Invoke-CheckedNativeCommand -Executable $pythonExecutable -Arguments @("-c", $verifyApiScript, $vspExecutableDirectory) -CaptureOutput)
    $versionLine = $verifyOutput | ForEach-Object { $_.ToString().Trim() } | Where-Object { $_.StartsWith($VersionMarker) } | Select-Object -Last 1
    if ([string]::IsNullOrWhiteSpace($versionLine)) {
        throw "OpenVSP verification did not report its version."
    }
    $openVspVersion = $versionLine.Substring($VersionMarker.Length)

    Push-Location $repositoryRoot
    try {
        Invoke-CheckedNativeCommand -Executable $pythonExecutable -Arguments @("-m", "pip", "install", "-e", ".")

        $outputDirectory = Split-Path -Parent $outputPath
        New-Item -ItemType Directory -Path $outputDirectory -Force | Out-Null

        Invoke-CheckedNativeCommand -Executable $pythonExecutable -Arguments @(
            "-m", "douglas_dart.cli", "openvsp-build",
            "--config", $configurationRelativePath,
            "--output", $outputRelativePath
        )
    }
    finally {
        Pop-Location
    }

    if (-not (Test-Path -LiteralPath $outputPath -PathType Leaf)) {
        throw "The Candidate B VSP3 file was not created: $outputPath"
    }
    $outputFile = Get-Item -LiteralPath $outputPath
    if ($outputFile.Length -le 0) {
        throw "The Candidate B VSP3 file is empty: $outputPath"
    }

    # Prove that the saved artifact is readable by a fresh model load through the
    # real API, then return geometry identities for the final human-readable report.
    $readBackScript = @'
import json
import sys
import openvsp

model_path = sys.argv[1]
openvsp.ClearVSPModel()
openvsp.ReadVSPFile(model_path)
openvsp.Update()

errors = []
if hasattr(openvsp, "GetNumTotalErrors") and hasattr(openvsp, "PopLastError"):
    while openvsp.GetNumTotalErrors() > 0:
        error = openvsp.PopLastError()
        if hasattr(error, "GetErrorString"):
            errors.append(str(error.GetErrorString()))
        else:
            errors.append(str(error))
if errors:
    raise RuntimeError("OpenVSP read-back errors: " + " | ".join(errors))

geometry_ids = list(openvsp.FindGeoms())
if not geometry_ids:
    raise RuntimeError("No geometry objects were loaded from the generated VSP3 file")

payload = {
    "version": str(openvsp.GetVSPVersion()).replace("OpenVSP", "").strip(),
    "geometries": [
        {"id": str(geometry_id), "name": str(openvsp.GetGeomName(geometry_id))}
        for geometry_id in geometry_ids
    ],
}
print("__DOUGLAS_DART_OPENVSP_GEOMETRY__=" + json.dumps(payload, separators=(",", ":")))
'@
    $readBackOutput = @(Invoke-CheckedNativeCommand -Executable $pythonExecutable -Arguments @("-c", $readBackScript, $outputFile.FullName) -CaptureOutput)
    $geometryLine = $readBackOutput | ForEach-Object { $_.ToString().Trim() } | Where-Object { $_.StartsWith($GeometryMarker) } | Select-Object -Last 1
    if ([string]::IsNullOrWhiteSpace($geometryLine)) {
        throw "OpenVSP read-back verification did not report loaded geometry."
    }
    $geometryData = $geometryLine.Substring($GeometryMarker.Length) | ConvertFrom-Json
    $geometries = @($geometryData.geometries)
    if ($geometryData.version -ne $ExpectedOpenVspVersion) {
        throw "OpenVSP version changed during read-back: expected $ExpectedOpenVspVersion, got $($geometryData.version)."
    }
    if ($geometries.Count -eq 0) {
        throw "No geometry objects were returned by OpenVSP read-back verification."
    }

    Write-Host ""
    Write-Host "Candidate B OpenVSP generation succeeded."
    Write-Host "OpenVSP version: $openVspVersion"
    Write-Host "Configuration path: $configurationPath"
    Write-Host "Output path: $($outputFile.FullName)"
    Write-Host "Output file size: $($outputFile.Length) bytes"
    Write-Host "Loaded geometries: $($geometries.Count)"
    foreach ($geometry in $geometries) {
        Write-Host "  - $($geometry.name) [$($geometry.id)]"
    }
}
catch {
    [Console]::Error.WriteLine("Candidate B OpenVSP generation failed: $($_.Exception.Message)")
    exit 1
}
