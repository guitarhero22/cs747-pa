echo "Encoding..."
python encoder.py --grid $args[0] | set-content mdpFile -Encoding Ascii
# echo "Pilot run..."
# python planner.py --mdp mdpFile --algorithm $args[1]
# echo "pilo run donek"
# $null = $Host.UI.RawUI.ReadKey('NoEcho,IncludeKeyDown');
echo "Final Run..."
$algo = $args[1]
Measure-Command {python planner.py --mdp mdpFile --algorithm $algo | set-content policy -Encoding Ascii}
echo "Finding Path"
python decoder.py --grid $args[0] --value_policy policy | set-content path -Encoding Ascii
echo "Visualize Grid..."
python visualize.py $args[0]
$null = $Host.UI.RawUI.ReadKey('NoEcho,IncludeKeyDown');
echo "Visualize Path..."
python visualize.py $args[0] path