include $(TOPDIR)/rules.mk

PKG_NAME:=mps_qmi_test
PKG_VERSION:=1.0.0
PKG_RELEASE:=1

#PKG_SOURCE_DIR:=$(CURDIR)/../src
PKG_SOURCE_DIR:=$(TOPDIR)/customer/src/$(PKG_NAME)
PKG_BUILD_DIR:=$(BUILD_DIR)/$(PKG_NAME)

include $(INCLUDE_DIR)/package.mk

define Package/$(PKG_NAME)
	SECTION:=utils
	CATEGORY:=Utilities
	TITLE:=MPs QMI Test
	DEPENDS:=+diag +qmiservices +qmiidl +qmi +qmi-framework +libqcmap_client +libqcmaputils +libqmi +libqmi-ip +dsutils
endef

define Package/$(PKG_NAME)/description
	MPs QMI Test utility for Telit modules.
endef

define Build/Prepare
	mkdir -p $(PKG_BUILD_DIR)
	cp -r $(PKG_SOURCE_DIR)/* $(PKG_BUILD_DIR)/
endef

define Build/Configure
	(cd $(PKG_BUILD_DIR); \
	autoreconf -fi; \
	./configure \
		--host=$(GNU_TARGET_NAME) \
		--prefix=/usr \
		CFLAGS="$(TARGET_CFLAGS)" \
		LDFLAGS="$(TARGET_LDFLAGS)")
endef

define Build/Compile
	$(MAKE) -C $(PKG_BUILD_DIR)
endef

define Package/$(PKG_NAME)/install
	$(INSTALL_DIR) $(1)/usr/bin
	$(INSTALL_BIN) $(PKG_BUILD_DIR)/mps_qmi_test $(1)/usr/bin/

	$(INSTALL_DIR) $(1)/etc/init.d
	$(INSTALL_BIN) ./files/$(PKG_NAME).init $(1)/etc/init.d/$(PKG_NAME).init	
endef

$(eval $(call BuildPackage,$(PKG_NAME)))
