# Google API Setup for Textbook Generation

This project uses Google Cloud APIs for textbook content generation instead of OpenAI. Follow these steps to set it up:

## 1. Create Google Cloud Project

1. Go to [Google Cloud Console](https://console.cloud.google.com/)
2. Create a new project or select an existing one
3. Enable the following APIs:
   - Cloud Natural Language API
   - Text-to-Speech API (if needed)

## 2. Create Service Account

1. Go to "IAM & Admin" > "Service Accounts"
2. Click "Create Service Account"
3. Name it (e.g., "textbook-generator")
4. Grant the following roles:
   - `Natural Language API User`
   - `Text-to-Speech User` (if using TTS features)

## 3. Create Service Account Key

1. In the service account details, go to "Keys" tab
2. Click "Add Key" > "Create new key"
3. Select "JSON" format
4. Download the key file

## 4. Configure the Application

1. Rename the downloaded key file to `google-service-account-key.json`
2. Place it in the project root directory (same level as backend/ folder)
3. Update your `.env` file:

```
GOOGLE_APPLICATION_CREDENTIALS=google-service-account-key.json
API_BASE_URL=http://localhost:8000
FRONTEND_PORT=3000
BACKEND_PORT=8000
```

## 5. Install Dependencies

```bash
cd backend
pip install -r requirements.txt
```

## 6. Run the Application

```bash
cd backend
uvicorn main:app --reload --port 8000
```

## 7. Alternative: Use Environment Variable

Instead of placing the key file in the project directory, you can:

1. Set the environment variable to point to your key file anywhere on your system:
   ```
   GOOGLE_APPLICATION_CREDENTIALS=/path/to/your/service-account-key.json
   ```

2. Or set it directly in your terminal:
   ```bash
   export GOOGLE_APPLICATION_CREDENTIALS="/path/to/your/service-account-key.json"
   ```

## Important Notes

- Keep your service account key file secure and never commit it to version control
- The service account key file should never be shared publicly
- For production deployments, consider using Google Cloud's built-in authentication mechanisms instead of service account key files

## Troubleshooting

If you get authentication errors:
1. Verify the service account key file exists and is accessible
2. Check that the required APIs are enabled in your Google Cloud project
3. Ensure the service account has the necessary permissions
4. Verify the path in the `GOOGLE_APPLICATION_CREDENTIALS` environment variable is correct