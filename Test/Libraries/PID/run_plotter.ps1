$scriptPath = Split-Path -Parent $MyInvocation.MyCommand.Definition
octave --persist "$scriptPath\octave_plotter.m"