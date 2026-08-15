px4_add_module(
	MODULE systemcmds__nvx_security
	MAIN nvx_security
	SRCS
		nvx_security.cpp
	DEPENDS
		nvx_security
)
