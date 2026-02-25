include $(TOPDIR)/rules.mk

PKG_NAME:=mps_atcmd_test
PKG_VERSION:=1.0
PKG_RELEASE:=1
PKG_FIXUP:=autoreconf
SOURCE_DIR:=$(TOPDIR)/customer/src/$(PKG_NAME)

include $(INCLUDE_DIR)/package.mk

TARGET_CFLAGS += \
	$(QMIFRAMEWORK_CFLAGS) \
	$(QMI_CFLAGS) \
	$(DIAG_CFLAGS) \
	$(DSUTILS_CFLAGS) \
	$(CFLAGS) \
	-I$(STAGING_DIR)/usr/include/diag \
	-I$(STAGING_DIR)/usr/include/dsutils \
	-I$(STAGING_DIR)/usr/include/qcmap_client \
	-I$(STAGING_DIR)/usr/include/qmi

TARGET_LDFLAGS += \
	-L$(STAGING_DIR)/usr/lib \
	-ldiag \
	-ldsutils \
	-lqmi_cci \
	-lqmiservices \
	-lqcmap_client

DEPENDS_MODULES += \
	+diag \
	+dsutils \
	+qmi \
	+qmi-framework \
	+libqcmap_client \
	+libqcmaputils \
	+libqmi-ip \
	+qmiservices \
	+liblog \
	+libcutils \
	+glib2

define Package/$(PKG_NAME)
	SECTION:=TELIT
	CATEGORY:=TELIT_Sample_Code
	TITLE:=Send ATC and receive response
	DEPENDS:=$(DEPENDS_MODULES)
endef

define Package/$(PKG_NAME)/description
	This is MPs AT Command Test sample package
endef

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	$(CP) $(SOURCE_DIR)/* $(PKG_BUILD_DIR)/
endef

define Build/Compile
	$(MAKE) -C $(PKG_BUILD_DIR) \
		CC="$(TARGET_CC)" \
		CFLAGS="$(TARGET_CFLAGS)" \
		LDFLAGS="$(TARGET_LDFLAGS)"
endef

define Package/$(PKG_NAME)/install
	$(INSTALL_DIR) $(1)/usr/bin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/$(PKG_NAME) $(1)/usr/bin/

	$(INSTALL_DIR) $(1)/data
	$(INSTALL_BIN) ./files/send_at_cmd.sh $(1)/data/send_at_cmd.sh	
	$(INSTALL_BIN) ./files/at_commands_list $(1)/data/at_commands_list	
endef

$(eval $(call BuildPackage,$(PKG_NAME)))