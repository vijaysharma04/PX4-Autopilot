#include <lib/nvx_security/nvx_security.h>
#include <px4_platform_common/module.h>

#include <cstring>

static void print_usage()
{
	PRINT_MODULE_DESCRIPTION("Controls temporary NVX USB service access. Service mode is RAM-only and resets at boot.");
	PRINT_MODULE_USAGE_NAME("nvx_security", "system");
	PRINT_MODULE_USAGE_COMMAND("status");
	PRINT_MODULE_USAGE_COMMAND_DESCR("service on", "Enable temporary USB service access");
	PRINT_MODULE_USAGE_COMMAND_DESCR("service off", "Return USB to Production restrictions");
}

extern "C" __EXPORT int nvx_security_main(int argc, char *argv[])
{
	if (argc == 2 && !strcmp(argv[1], "status")) {
		PX4_INFO("NVX Security: %s", nvx_security::mode_name());
		return 0;
	}

	if (argc == 3 && !strcmp(argv[1], "service")) {
		if (!strcmp(argv[2], "on")) {
			nvx_security::set_service_mode(true);
			PX4_WARN("NVX Security: Service");
			return 0;
		}

		if (!strcmp(argv[2], "off")) {
			nvx_security::set_service_mode(false);
			PX4_INFO("NVX Security: Production");
			return 0;
		}
	}

	print_usage();
	return 1;
}
