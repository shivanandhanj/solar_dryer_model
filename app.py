from flask import Flask, request, jsonify
import joblib
import pandas as pd
from flask_cors import CORS

app = Flask(__name__)
CORS(app)

# Load trained model, scaler, and encoders
model = joblib.load("drying_time_model.pkl")
scaler = joblib.load("scaler.pkl")
label_encoders = joblib.load("label_encoders.pkl")

@app.route("/predict", methods=["POST"])
def predict():
    data = request.json  # Receive JSON input
    
    try:
        # Encode categorical inputs
        encoded = {
            "produce_type": label_encoders["produce_type"].transform([data["produce_type"]])[0],
            "name": label_encoders["name"].transform([data["name"]])[0],
            "method": label_encoders["method"].transform(["Hot-Air"])[0],  # fixed
            "drying_temperature_C": data["drying_temperature_C"]  # ✅ use temp instead of time
        }
        
        df = pd.DataFrame([encoded])
        df_scaled = scaler.transform(df)
        prediction = model.predict(df_scaled)[0]
        
        return jsonify({
            
            "predicted_drying_time_min": round(float(prediction), 2)  # ✅ corrected
        })
    
    except Exception as e:
        return jsonify({"error": str(e)})

if __name__ == "__main__":
    app.run(debug=True, port=5000)
