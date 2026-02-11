# Submission

## Preparing for Submission

After you are satisfied with your policy, you will need to prepare a Docker image for submission.

```bash
$ mkdir -p docker/my_policy_node
$ cp docker/aic_model/Dockerfile docker/my_policy_node/
```

Open the Dockerfile and add your policy node to the build instructions:

```dockerfile
# Add other local dependencies
COPY my_policy_node /ws_aic/src/aic/my_policy_node # <-- Add this line
```

Open `docker/docker-compose.yaml` and update the model service configuration to use your Dockerfile and policy:

`docker/docker-compose.yaml`:

```yaml
model:
	image: localhost/aic/aic_model
	build:
		dockerfile: docker/my_policy_node/Dockerfile # <-- replace this line
		context: ..
	command: --ros-args -p policy:=my_policy_node.WaveArm # <-- and this line
```

Build the image:

```bash
$ docker compose build model
```

Test that everything works:

```bash
$ docker compose up
```

This will run your model and the evaluator in an environment that replicates the submission portal:

- External network access is restricted.
- Zenoh ACLs will be employed to restrict what the policy node can interact with.
- Shared memory is disabled.

Make sure that your policy works, then export a tarball of your image.

```bash
$ docker save localhost/aic/my_policy_node | gzip > my_policy_node.tar.gz
```

You are now ready to submit the tarball of your image to the participation portal.
*TODO*