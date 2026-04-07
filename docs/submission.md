# Submission guidelines

## Introduction

Welcome to the **AI for Industry Challenge**. This document outlines the technical requirements for packaging, containerizing, and uploading your solutions for evaluation. Following these steps ensures your model runs in our automated evaluation environment exactly as it does on your local machine.

> [!IMPORTANT]
> To complete the registry upload, you must have the credentials provided in your **onboarding email** sent to team leaders. These include your unique AWS access credentials, and the ECR Repository URI assigned to your team.

---

## 1. Prepare and Build Your Image

All submissions must be containerized using OCI-compliant image builder like Docker or Podman. Organize your project by placing all policy logic and dependency requirements directly within your custom policy package.

If you don't have any additional packages or dependencies, you can keep your policy code in [policy.py](../aic_model/aic_model/policy.py) and then re-use the `aic_model` directory with its [Dockerfile](../docker/aic_model/Dockerfile). In this case, update the Dockerfile and change `CMD ["--ros-args", "-p", "policy:=aic_example_policies.ros.CheatCode", "-p", "use_sim_time:=true"]` to `CMD ["--ros-args", "-p", "policy:=aic_model.MyPolicy", "-p", "use_sim_time:=true"]`, and skip to the [Build the Image](#build-the-image) section.

It is highly recommended to use the example aic_model Dockerfile as a starting point.

```bash
mkdir -p docker/my_policy
cp docker/aic_model/Dockerfile docker/my_policy/
```

Then modify `docker/my_policy/Dockerfile` to add your custom policy package:

```dockerfile
# Add other dependencies
COPY my_policy_node /ws_aic/src/aic/my_policy_node # <-- Add this line
```

Edit the `CMD` to run your policy:

```dockerfile
CMD ["--ros-args", "-p", "policy:=my_policy_node.MyPolicy"]
CMD ["--ros-args", "-p", "policy:=my_policy_node.MyPolicy", "-p", "use_sim_time:=true"]
```

### Update `docker-compose.yaml`

Open `docker/docker-compose.yaml` and update the model service configuration to use your Dockerfile and policy:

```yaml
    model:
        image: my-solution:v1
        build:
            dockerfile: docker/my_policy_node/Dockerfile # <-- replace this line
            context: ..
```

### Build the Image

To build your submission image, run the following command from the **root directory** directory:

```bash
docker compose -f docker/docker-compose.yaml build model
```

### Verify Locally

Before pushing your image to our servers, you must verify that the container initializes correctly and handles data as expected.

You can run the evaluation locally using `docker compose`:

```bash
docker compose -f docker/docker-compose.yaml up
```

> [!WARNING]
> Do not skip local verification. If your container fails to start or crashes during the local evaluation, it will be automatically rejected by the submission portal, which may count against your daily submission limit.

> [!IMPORTANT]
> A description of the Zenoh access controls used to prevent minimal "cheating" solutions that simply subscribe to the simulator's internal data structures are described in the [Access Control](access_control.md) document.

---

## 2. Upload Your Image to Our Registry

We use Amazon Elastic Container Registry (ECR) to host team OCI images. You will need to have [AWS CLI](https://docs.aws.amazon.com/cli/latest/userguide/getting-started-install.html) installed.

### Authenticate

Configure your local environment by following these steps, using the credentials provided in your onboarding email sent to team leaders:

#### A. Configure your AWS Profile
Run the following command, replacing `<team_name>` with the slug provided in your email (e.g., `team123`):

```bash
aws configure --profile <team_name>
```

When prompted, enter the following details:

- **Access Key ID:** (Copy from email)
- **Secret Access Key:** (Copy from email)
- **Default region name:** us-east-1
- **Default output format:** json (or press Enter for default)

#### B. Set the Environment Variable

Point your shell to the new profile so subsequent commands use the correct credentials:

```bash
export AWS_PROFILE=<team_name>
```

#### C. Authenticate with the Registry

Finally, authenticate your local Docker client with our private registry:

```bash
aws ecr get-login-password --region us-east-1 | docker login --username AWS --password-stdin 973918476471.dkr.ecr.us-east-1.amazonaws.com
```

### Tag Your Image

You must tag your local image to match the remote repository URI provided to your team. Replace the dummy URI below with your specific team URI:

```bash
docker tag localhost/my-solution:v1 973918476471.dkr.ecr.us-east-1.amazonaws.com/aic-team/<team_name>:v1
```

> [!IMPORTANT]
> Image tags in our ECR registry are immutable. You cannot overwrite an existing tag. For each new submission or build, you must increment your version tag (e.g., :v2, :v3) or use a unique identifier like a Git commit SHA. If you try to push an image with a tag that already exists in the registry, the push will fail.

### Push Your Image

Upload the tagged image to the challenge registry:

```bash
docker push 973918476471.dkr.ecr.us-east-1.amazonaws.com/aic-team/<team_name>:v1
```

---

## 3. Register Your Submission

Simply pushing the image to ECR does not trigger the evaluation. You must notify the platform that a new version is ready for scoring.

> [!NOTE]
> The submission portal will be open shortly. The login credentials for the submission portal will be emailed to the team leaders by the end of March.

1. Copy the full Image URI you just pushed (e.g., `973918476471.dkr.ecr.us-east-1.amazonaws.com/aic-team/<team_name>:v1`).
2. Log in to the submission portal.
3. Click on the `AI for Industry Challenge` and then go to `Submit`.
4. Select the `Qualification` phase and paste the URI into the submission `OCI Image` field.
5. Click `Submit` to proceed.

---

### 4. Monitor Your Evaluation

After registering your OCI Image URI, our orchestration platform spins up your container into a dedicated, isolated evaluation environment. This process is automated, but you can track its lifecycle through the portal's monitoring dashboard.

#### Accessing the Dashboard
1. Navigate to the **My Submissions** page in the portal.
2. Apply the `Qualification` filter to the "Phase" dropdown to see your current entries.
3. Locate your most recent submission at the top of the table.

#### Evaluation Lifecycle

The **Status** column provides a real-time status of your container's journey through our evaluation cluster. Understanding these states is key to managing your daily submission limit.

| Status | Technical Context |
| :--- | :--- |
| **Submitted** | The platform has received your Image URI. |
| **Queued** | Your submission is in the execution buffer. It is waiting for an available evaluation node in the cluster. |
| **Running** | Your image has been pulled from ECR, and the ROS 2 nodes are currently executing the challenge logic in the simulation environment. |
| **Finished** | The evaluation reached a natural conclusion. Your success metrics have been calculated and are now visible on the Leaderboard. |
| **Failed** | The container exited prematurely. This usually indicates a runtime crash (e.g., Python `ImportError`), a missing dependency, or a system timeout. |

> [!TIP]
> Depending on cluster load and the complexity of your policy, the transition from **Queued** to **Finished** typically takes **5 to 15 minutes**. You do not need to resubmit if the status is "Queued" or "Running"; simply refresh the page to see the latest state.

---

## FAQs

**I cannot use the example dockerfile**: The example dockerfile assumes that you are using `aic_model` to run your policy. If you are not using `aic_model`, you can [create a custom dockerfile](./custom_dockerfile.md).

**My push failed with "no basic auth credentials"**: Your Docker login session has likely expired. ECR login tokens are valid for 12 hours. Repeat the [Authenticate](#authenticate) step in Section 2.

**Where can I see my results?** All your past results and logs can be consulted in the "My submissions" section of the portal. You can also visit the Leaderboard to compare your results against the rest of the teams.

**Who can submit?** Any team member can submit on behalf of the team. Note that the daily submission limit applies to the team as a whole, not per individual.

**Can I submit multiple times?** Yes. However, you are limited to 1 submission per day. There is no limit on the total number of submissions you can make throughout the duration of the competition.

---

## Questions?

- **Issues**: Report problems via [GitHub Issues](https://github.com/intrinsic-dev/aic/issues)
- **Community**: Join discussions at [Open Robotics Discourse](https://discourse.openrobotics.org/c/competitions/ai-for-industry-challenge/)
