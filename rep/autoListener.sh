while true; do
	inotifywait /home/bharat/Downloads 
	python3 checkFile.py
done
