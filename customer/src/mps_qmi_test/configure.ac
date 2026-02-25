
AC_PREREQ(2.61)
AC_INIT([mps_qmi_test],1.0.0)
AM_INIT_AUTOMAKE([foreign])
AM_MAINTAINER_MODE
AC_CONFIG_SRCDIR([mps_qmi_test.c])
AC_CONFIG_HEADERS([config.h])
AC_CONFIG_MACRO_DIR([m4])

# Checks for programs.
AC_PROG_CC
AM_PROG_CC_C_O
AC_PROG_LIBTOOL
AC_PROG_AWK
AC_PROG_CPP
AC_PROG_INSTALL
AC_PROG_LN_S
AC_PROG_MAKE_SET
PKG_PROG_PKG_CONFIG

# Checks for libraries.
PKG_CHECK_MODULES([QMIFRAMEWORK], [qmi-framework])
AC_SUBST([QMIFRAMEWORK_CFLAGS])
AC_SUBST([QMIFRAMEWORK_LIBS])

PKG_CHECK_MODULES([QMI], [qmi])
AC_SUBST([QMI_CFLAGS])
AC_SUBST([QMI_LIBS])

AC_ARG_WITH([kernel],
      AC_HELP_STRING([--with-kernel=@<:@dir@:>@],
                     [Specify the location of the Linux kernel headers]),
                     [kerneldir=$withval],
                      with_kernel=no)

AC_ARG_WITH(afs-telit,
[AS_HELP_STRING([],[])],
[
  PKG_CHECK_MODULES([AFSTELIT], [afs-telit])
  AC_SUBST([AFSTELIT_CFLAGS])
  AC_SUBST([AFSTELIT_LIBS])
])

if test "x$with_kernel" != "xno"; then
   CFLAGS="${CFLAGS} -I${kerneldir}/include -I${kerneldir}/arch/arm/include"
fi

AC_ARG_ENABLE(project,
  [AS_HELP_STRING([--enable-project=PROJECT], [Specify the target product to build])],
  [PROJECT=$enableval],
  [PROJECT=none]
)

AC_ARG_ENABLE([afs-telit], AS_HELP_STRING([],[]), [USE_AFS_TELIT=true], [USE_AFS_TELIT=false])

AC_CONFIG_FILES([ \
        Makefile \
        mps_qmi_test.pc \
        ])

AM_CONDITIONAL([IS_FE910], [test "${PROJECT:0:5}" = "FE910"])
AM_CONDITIONAL([IS_FN920], [test "${PROJECT:0:5}" = "FN920"])
AM_CONDITIONAL([IS_PVR81], [test "${PROJECT:0:5}" = "PVR81"])

AM_CONDITIONAL([IS_FN990], [test "${PROJECT:0:5}" = "FN990"])
AM_CONDITIONAL([IS_FE990], [test "${PROJECT:0:5}" = "FE990"])
AM_CONDITIONAL([IS_FN991], [test "${PROJECT:0:5}" = "FN991"])
AM_CONDITIONAL([IS_FI990], [test "${PROJECT:0:5}" = "FI990"])
AM_CONDITIONAL([USE_AFS_TELIT], [$USE_AFS_TELIT])

AM_COND_IF([USE_AFS_TELIT], [CPPFLAGS="${CPPFLAGS} -DCONFIG_SUPPORT_AFS_TELIT"])

AC_OUTPUT

