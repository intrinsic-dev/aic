# Submission guidelines

## Introduction

Welcome to the **AI for Industry Challenge**. This document outlines the technical requirements for packaging, containerizing, and uploading your solutions for evaluation. Following these steps ensures your model runs in our automated evaluation environment exactly as it does on your local machine.

> [!IMPORTANT]
> To complete the registry upload, you must have the credentials provided in your **onboarding email**. These include your unique AWS access credentials, and the ECR Repository URI assigned to your team.

---

## 1. Prepare and Build Your Image

All submissions must be containerized using OCI-compliant image builder like Docker or Podman. Your project structure should place your model logic and requirements within the `aic_model` directory.

### Customize Your Dockerfile (Optional)

If you need to add custom packages or dependencies, you can create a custom Dockerfile:

```bash
mkdir -p docker/my_policy
cp docker/aic_model/Dockerfile docker/my_policy/
```

Then modify `docker/my_policy/Dockerfile` to add your custom policy package:

```dockerfile
# Add your custom policy package
COPY my_policy_node /ws_aic/src/aic/my_policy_node
```

If your policy requires additional Python packages, add them to `pixi.toml`:

```toml
[dependencies]
# ... existing dependencies ...
torch = ">=2.0.0"
numpy = ">=1.24.0"
```

### Build the Image

To build your submission image, run the following command from the **root directory** of your project to ensure the build context includes all necessary files:

```bash
docker build -t my-solution:v1 -f docker/aic_model/Dockerfile .
```

If you created a custom Dockerfile, use that path instead:

```bash
docker build -t my-solution:v1 -f docker/my_policy/Dockerfile .
```

### Verify Locally

Before pushing your image to our servers, you must verify that the container initializes correctly and handles data as expected.

You can run the evaluation locally using docker compose:

```bash
docker compose -f docker/docker-compose.yaml up
```

> [!WARNING]
> Do not skip local verification. If your container fails to start or crashes during the local evaluation, it will be automatically rejected by the submission portal, which may count against your daily submission limit.

---

## 2. Upload Your Image to Our Registry

We use Amazon Elastic Container Registry (ECR) to host team OCI images.

### Authenticate

First authenticate to AWS with the credentials provided in your onboarding email:

```bash
aws configure --profile <team_name_slug>
```
This will prompt you to enter your AWS Access Key ID, Secret Access Key, region (use `us-east-1`), and output format (you can use the sensible default by pressing enter).

Set that `AWS_PROFILE` to your team name provided in your onboarding email:

```bash
export AWS_PROFILE=<team_name>
```

For example, if your team name slug is `team123`, you would run:

```bash
aws configure --profile team123
# Complete the prompts with your credentials
export AWS_PROFILE=team123
```

Then, authenticate your local client with our private registry.

```bash
aws ecr get-login-password --region us-east-1 | docker login --username AWS --password-stdin 123456789.dkr.ecr.us-east-1.amazonaws.com
```

### Tag Your Image

You must tag your local image to match the remote repository URI provided to your team. Replace the dummy URI below with your specific team URI:

```bash
docker tag my-solution:v1 123456789.dkr.ecr.us-east-1.amazonaws.com/aic-team-name:v1
```

### Push Your Image

Upload the tagged image to the challenge registry:

```bash
docker push 123456789.dkr.ecr.us-east-1.amazonaws.com/aic-team-name:v1
```

---

## 3. Register Your Submission

Simply pushing the image to ECR does not trigger the evaluation. You must notify the platform that a new version is ready for scoring.

1. Copy the full Image URI you just pushed (e.g., `123456789.dkr.ecr.us-east-1.amazonaws.com/aic-team-name:v1`).
2. Log in to the [aiforindustrychallenge.ai](https://aiforindustrychallenge.ai) portal.
3. Click on the `AI for Industry Challenge` and then go to `Submit`.
4. Select the `Train your model` phase and paste the URI into the submission `OCI Image` field.
5. Click `Submit` to proceed.

---

## FAQs

**Can I submit multiple times?** Yes. However, you are limited to 1 submission per day. There is no limit on the total number of submissions you can make throughout the duration of the competition.

**My push failed with "no basic auth credentials"**: Your Docker login session has likely expired. ECR login tokens are valid for 12 hours. Repeat the [Authenticate](#authenticate) step in Section 2.

**Where can I see my results?** All your past results and logs can be consulted in the "My submissions" section of the portal. You can also visit the Leaderboard to compare your results against the rest of the teams.

---

## Questions?

- **Issues**: Report problems via [GitHub Issues](https://github.com/intrinsic-dev/aic/issues)
- **Community**: Join discussions at [Open Robotics Discourse](https://discourse.openrobotics.org/c/competitions/ai-for-industry-challenge/)
