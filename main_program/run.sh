#IFS='\n'       # make newlines the only separator
rm -rf results.csv
for j in $(cat ./input-parameters.csv | tail -n +2 -)
do
    echo "$j"
    #echo "Done"
    python3 simulate-without-plots.py "$j"
done
