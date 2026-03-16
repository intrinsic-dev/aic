#!/usr/bin/env python3
"""General-purpose xacro expansion service for the AIC eval stack."""

from pathlib import Path
import subprocess

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from aic_engine_interfaces.srv import ExpandXacro
import rclpy
from rclpy.node import Node


class XacroExpanderNode(Node):
    """Serve xacro expansion requests from the already-configured eval environment."""

    def __init__(self) -> None:
        super().__init__("xacro_expander")
        self.create_service(ExpandXacro, "expand_xacro", self._handle_expand_xacro)

    def _handle_expand_xacro(
        self,
        request: ExpandXacro.Request,
        response: ExpandXacro.Response,
    ) -> ExpandXacro.Response:
        # Validate required request fields.
        package_name = request.package_name.strip()
        relative_path = request.relative_path.strip()

        if not package_name:
            response.success = False
            response.message = "package_name is required"
            return response
        if not relative_path:
            response.success = False
            response.message = "relative_path is required"
            return response

        # Resolve the package's share directory via the ament index.
        try:
            package_share = Path(get_package_share_directory(package_name)).resolve()
        except PackageNotFoundError as exc:
            response.success = False
            response.message = str(exc)
            return response

        # Prevent path traversal: the resolved path must stay inside the share dir.
        candidate = (package_share / relative_path).resolve()
        try:
            candidate.relative_to(package_share)
        except ValueError:
            response.success = False
            response.message = "relative_path must stay within the package share directory"
            return response

        # Verify the xacro source file exists.
        if not candidate.is_file():
            response.success = False
            response.message = f"xacro file not found: {candidate}"
            return response

        # Run the xacro CLI to expand the file with the provided arguments.
        cmd = ["xacro", str(candidate), *list(request.xacro_arguments)]
        result = subprocess.run(
            cmd,
            check=False,
            capture_output=True,
            text=True,
        )
        if result.returncode != 0:
            response.success = False
            response.message = result.stderr.strip() or f"xacro failed with code {result.returncode}"
            return response

        # Guard against xacro producing empty output (e.g. missing template content).
        xml = result.stdout
        if not xml.strip():
            response.success = False
            response.message = "xacro returned an empty XML string"
            return response

        response.success = True
        response.xml = xml
        response.message = "xacro expansion succeeded"
        return response


def main() -> None:
    rclpy.init()
    node = XacroExpanderNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
