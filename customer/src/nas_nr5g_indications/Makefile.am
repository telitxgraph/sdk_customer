AM_CFLAGS = -g -O2 \
	-fno-inline \
	-fno-short-enums \
	-fpic \
	$(CFLAGS) \
	$(QMIFRAMEWORK_CFLAGS) \
	$(QMI_CFLAGS) \
	-I./ \
	-Wall -Wextra -Werror -D_FORTIFY_SOURCE=2 \
	-Wformat=2 -Wformat-security \
	-Wmissing-declarations -Wnull-dereference -Wstrict-overflow -Wtrampolines \
	-Wmissing-prototypes -Wstrict-prototypes \
	-Wno-unused-parameter -Wno-unused-variable -Wno-unused-function \
	-Wno-format-truncation

nas_nr5g_indications_SOURCES = \
	nas_nr5g_indications.c \
	nas_nr5g_indications_config.c

requiredlibs = $(QMIFRAMEWORK_LIBS) $(QMI_LIBS)

bin_PROGRAMS = nas_nr5g_indications

nas_nr5g_indications_LDADD = $(requiredlibs)

nas_nr5g_indications_LDFLAGS = -lrt -lpthread -llog \
	$(QMIFRAMEWORK_LIBS) $(QMI_LIBS) \
	-L$(STAGING_DIR)/usr/lib \
	-lqmiidl -lqmiservices -lqmi_cci \
	-lqmi_client_qmux \
	-ldiag

nas_nr5g_indications_CC = @cc@
