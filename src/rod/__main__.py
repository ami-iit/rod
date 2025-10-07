import argparse
import logging
import sys
from typing import NoReturn

from rod import logging as rodlogging

try:
    from rod._version import __version__
except ImportError:
    # Fallback for development installations
    try:
        import importlib.metadata

        __version__ = importlib.metadata.version("rod")
    except importlib.metadata.PackageNotFoundError:
        __version__ = "unknown"


def main() -> NoReturn:
    """
    Main function of the ROD command line interface.
    """
    parser = argparse.ArgumentParser(
        prog="rod",
        description="ROD: The ultimate Python tool for RObot Descriptions processing.\n"
        "Load, parse, convert, and manipulate robot description files (SDF/URDF).",
        epilog="Examples:\n"
        "  rod -f robot.urdf --show           # Display robot model structure\n"
        "  rod -f robot.sdf -o robot.urdf     # Convert SDF to URDF\n"
        "  rod -f robot.urdf -o robot.sdf     # Convert URDF to SDF\n"
        "  rod --version                      # Show version information",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    # Version.
    parser.add_argument(
        "-V",
        "--version",
        action="version",
        version=f"%(prog)s {__version__}",
        help="Show version information and exit.",
    )

    # Verbose output.
    parser.add_argument(
        "-vv",
        "--verbose",
        action="store_true",
        help="Enable verbose output with detailed logging information.",
    )

    # File to parse.
    parser.add_argument(
        "-f",
        "--file",
        type=str,
        metavar="PATH",
        help="Path to the robot description file to parse (supports .sdf, .urdf).",
        required=False,
    )

    # Option to display the parser file attributes.
    parser.add_argument(
        "-s",
        "--show",
        action="store_true",
        help="Display the parsed robot model structure and properties in a human-readable format.",
    )

    # Option to output a URDF or SDF file.
    parser.add_argument(
        "-o",
        "--output",
        type=str,
        metavar="PATH",
        help="Output file path for format conversion (.urdf or .sdf extension required).",
    )

    args = parser.parse_args()

    log_level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(level=log_level)

    from rod.urdf.exporter import UrdfExporter

    # Ensure file argument is provided if output or `show` is specified.
    if not args.file and (args.output or args.show):
        parser.error(
            "The `--file` argument is required when using `--output` or `--show`."
        )

    # Show the file attributes if no output file is specified.
    if args.file and not (args.output or args.show):
        args.show = True

    # Display the parsed file attributes.
    if args.file:
        from rod import Sdf

        try:
            sdf = Sdf.load(sdf=args.file)
        except Exception as e:
            rodlogging.error(f"Error loading file: {e}")
            sys.exit(1)

    if args.show:
        print(sdf.to_string())

    # Output the URDF or SDF file.
    if args.output:

        try:
            if args.output.endswith(".urdf"):
                with open(args.output, "w") as file:
                    file.write(UrdfExporter(pretty=True).to_urdf_string(sdf=sdf))

            elif args.output.endswith(".sdf"):
                with open(args.output, "w") as file:
                    file.write(sdf.serialize(pretty=True))

            else:
                rodlogging.error(
                    f"Unsupported output file extension for '{args.output}'. Supported extensions are '.urdf' and '.sdf'."
                )
                sys.exit(1)

        except Exception as e:
            rodlogging.exception(f"Error writing output file: {e}")
            sys.exit(1)

    # Exit successfully if we reach here
    sys.exit(0)


if __name__ == "__main__":
    main()
