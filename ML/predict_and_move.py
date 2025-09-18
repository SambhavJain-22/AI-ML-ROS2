import joblib
import time

# Load the model and encoders
clf = joblib.load("room_decision_tree.pkl")
le_time, le_task, le_dirt, le_target = joblib.load("label_encoders.pkl")

# Valid options
valid_times = le_time.classes_.tolist()
valid_tasks = le_task.classes_.tolist()
valid_dirt = le_dirt.classes_.tolist()

# Get user input
print("\n📋 Please enter task details for the robot:")
print(f"Available Time of Day options: {valid_times}")
time_input = input("⏰ Time of Day: ").strip().lower()

print(f"\nAvailable Task Types: {valid_tasks}")
task_input = input("📦 Task Type: ").strip().lower()

print(f"\nAvailable Dirt Levels: {valid_dirt}")
dirt_input = input("🧹 Dirt Level: ").strip().lower()

try:
    # Encode input
    X_input = [[
        le_time.transform([time_input])[0],
        le_task.transform([task_input])[0],
        le_dirt.transform([dirt_input])[0]
    ]]

    # Predict the room
    predicted_class = clf.predict(X_input)[0]
    predicted_room = le_target.inverse_transform([predicted_class])[0]

    # Simulate robot movement
    print(f"\n[🧠] Predicted room to visit: {predicted_room}")
    print(f"[🤖] Sending robot to {predicted_room}...")

    for i in range(3):
        time.sleep(1)
        print(f"  ...moving...{'.' * (i+1)}")

    print(f"[✅] Robot has reached the {predicted_room}.\n")

except ValueError as e:
    print(f"\n❌ Invalid input. Please enter values exactly as shown above.")
