"""불변 RTAB-Map localization release를 실행하기 전에 preflight를 수행한다."""

import argparse
import os
import sys

from stier_slam_core.map_release import ReleaseManager, ReleaseValidationError


def prepare_runtime_database(release_manifest, map_yaml, source_database, runtime_database):
    return ReleaseManager.prepare_runtime_database(
        release_manifest, map_yaml, source_database, runtime_database
    )


def prepare_runtime_config(source_config, runtime_config):
    return ReleaseManager.prepare_runtime_config(source_config, runtime_config)


def prepare_runtime_plan(
        source_config, runtime_config, mapping_database=None, mapping_runtime_root=None,
        mapping_session_dir=None, release_manifest=None,
        map_yaml=None, source_database=None, runtime_database=None):
    return ReleaseManager.plan_runtime_artifacts(
        source_config_path=source_config,
        runtime_config_path=runtime_config,
        mapping_database_path=mapping_database,
        mapping_runtime_root_path=mapping_runtime_root,
        mapping_session_dir_path=mapping_session_dir,
        release_manifest_path=release_manifest,
        map_yaml_path=map_yaml,
        source_database_path=source_database,
        runtime_database_path=runtime_database,
    )


def guard_main(argv=None, execvp=os.execvp):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--release-manifest")
    parser.add_argument("--map-yaml")
    parser.add_argument("--source-database")
    parser.add_argument("--runtime-database")
    parser.add_argument("--mapping-database")
    parser.add_argument("--mapping-runtime-root")
    parser.add_argument("--mapping-session-dir")
    parser.add_argument("--runtime-root")
    parser.add_argument("--runtime-bundle")
    parser.add_argument("--bundle-role", choices=("map-server", "rtabmap"))
    parser.add_argument("--source-config", required=True)
    parser.add_argument("--runtime-config")
    parser.add_argument("command", nargs=argparse.REMAINDER)
    arguments = parser.parse_args(argv)
    if arguments.command[:1] == ["--"]:
        arguments.command = arguments.command[1:]
    if not arguments.command:
        parser.error("a RTAB-Map command is required after --")
    try:
        if arguments.runtime_bundle:
            if (arguments.mapping_database or arguments.mapping_runtime_root
                    or arguments.mapping_session_dir or arguments.runtime_database
                    or arguments.runtime_config or not all((
                        arguments.release_manifest, arguments.map_yaml,
                        arguments.source_database, arguments.runtime_root,
                        arguments.bundle_role))):
                raise ReleaseValidationError(
                    "runtime bundle requires one root, one role, and no individual runtime paths"
                )
            ReleaseManager.prepare_localization_runtime_bundle(
                arguments.release_manifest,
                arguments.map_yaml,
                arguments.source_database,
                arguments.source_config,
                arguments.runtime_bundle,
                runtime_root_path=arguments.runtime_root,
                bundle_role=arguments.bundle_role,
            )
        else:
            if arguments.bundle_role or arguments.runtime_root:
                raise ReleaseValidationError("bundle role and runtime root require runtime bundle")
            if not arguments.runtime_config:
                raise ReleaseValidationError("runtime config is required outside bundle mode")
            plan = prepare_runtime_plan(
                arguments.source_config,
                arguments.runtime_config,
                mapping_database=arguments.mapping_database,
                mapping_runtime_root=arguments.mapping_runtime_root,
                mapping_session_dir=arguments.mapping_session_dir,
                release_manifest=arguments.release_manifest,
                map_yaml=arguments.map_yaml,
                source_database=arguments.source_database,
                runtime_database=arguments.runtime_database,
            )
            ReleaseManager.execute_runtime_plan(plan)
        execvp(arguments.command[0], arguments.command)
    except (OSError, ReleaseValidationError, ValueError) as error:
        print("error: {}".format(error), file=sys.stderr)
        return 2
    return 0
