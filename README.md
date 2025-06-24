# Hand Gesture Recognition Code

Uses Mediapipe's hand gesture recognizer.

To run the code, pip install mediapipe

Then run python3 classify_hand.py to use your webcam to classify whatever gestures you make. 

If you want to make your own gestures, just add a directory called hand_signs and put a folder in that directory for each static gesture you want. then fill those folders with different images. (for now they all have to end in .jpeg because I was lazy)
Once you have images for each hand sign, go ahead and run generate_reference_data.py to update the pickle file.
