# Run the `alog` this skill was written for, with any arguments: scripts/alog.ps1 info flight.bin
#
# Order: an installed `alog` whose --version is the pinned one; else uvx (or `uv tool run`);
# else pipx; else an installed `alog` of another version, with a warning; else print how to
# install it and exit 3 (the CLI's own "could not run" code). $Pin and $Ref are set by
# tools/build_skill.py from the package version: $Ref is the release tag, or main for a
# development version.
$Pin = '2.1.0'
$Ref = 'v2.1.0'
$Url = 'https://github.com/geoffreyheller/ardupilot-log-tools'
$Spec = "git+$Url@$Ref"

function Test-Command($Name) { [bool](Get-Command $Name -ErrorAction SilentlyContinue) }

if ((Test-Command alog) -and ((& alog --version 2>$null) -eq "ardupilot-log-tools $Pin")) {
    & alog @args; exit $LASTEXITCODE
}
if (Test-Command uvx) { & uvx --from $Spec alog @args; exit $LASTEXITCODE }
if (Test-Command uv) { & uv tool run --from $Spec alog @args; exit $LASTEXITCODE }
if (Test-Command pipx) { & pipx run --spec $Spec alog @args; exit $LASTEXITCODE }
if (Test-Command alog) {
    [Console]::Error.WriteLine("warning: using installed $(& alog --version 2>$null); this skill was written for $Pin")
    & alog @args; exit $LASTEXITCODE
}
[Console]::Error.WriteLine("alog $Pin is not installed, and neither uv nor pipx is available to run it.")
[Console]::Error.WriteLine("Install it with:  pip install `"$Spec`"   (Python 3.9+)")
exit 3
