echo "Encoding..."
python encoder.py --grid $1 > mdpFile
echo "Pilot run..."
# python planner.py --mdp mdpFile --algorithm $2
# read -n 1 -s
echo "Final Run..."
time python planner.py --mdp mdpFile --algorithm $2 > policy
echo "Finding Path"
python decoder.py --grid $1 --value_policy policy > path
echo "Visualize Grid..."
python visualize.py $1
read -n 1 -s
echo "Visualize Path..."
python visualize.py $1 path