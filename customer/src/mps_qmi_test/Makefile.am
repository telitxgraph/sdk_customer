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

mps_qmi_test_SOURCES = \
	mps_qmi_test.c \
	gms.c

requiredlibs = $(QMIFRAMEWORK_LIBS) $(QMI_LIBS)

bin_PROGRAMS = mps_qmi_test

mps_qmi_test_LDADD = $(requiredlibs)

mps_qmi_test_LDFLAGS = -lrt -lpthread -llog \
	$(QMIFRAMEWORK_LIBS) $(QMI_LIBS) \
	-L$(STAGING_DIR)/usr/lib \
	-lqmiidl -lqmiservices -lqmi_cci \
	-lqmi_client_qmux \
	-ldiag

mps_qmi_test_CC = @cc@
