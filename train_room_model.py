import pandas as pd
from sklearn.tree import DecisionTreeClassifier
from sklearn.preprocessing import LabelEncoder
from sklearn.model_selection import train_test_split
from sklearn.metrics import accuracy_score
import joblib

# Load dataset
df = pd.read_csv("room_dataset.csv")


# Encode categorical columns
le_time = LabelEncoder()
le_task = LabelEncoder()
le_dirt = LabelEncoder()
le_target = LabelEncoder()

df['time_encoded'] = le_time.fit_transform(df['Time of Day'])
df['task_encoded'] = le_task.fit_transform(df['Task Type'])
df['dirt_encoded'] = le_dirt.fit_transform(df['Dirt Level'])
df['target_encoded'] = le_target.fit_transform(df['Target Room'])

# Features and target
X = df[['time_encoded', 'task_encoded', 'dirt_encoded']]
y = df['target_encoded']

# Split and train model
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)
clf = DecisionTreeClassifier()
clf.fit(X_train, y_train)

# Evaluate
y_pred = clf.predict(X_test)
print(f"Model Accuracy: {accuracy_score(y_test, y_pred):.2f}")

# Save model and encoders
joblib.dump(clf, "room_decision_tree.pkl")
joblib.dump((le_time, le_task, le_dirt, le_target), "label_encoders.pkl")

print("[✔] Model and encoders saved.")
