%define kernel_version %KERNEL_VERSION%
%define kernel_build %KERNEL_BUILD%
%define kernel_release %KERNEL_RELEASE%
%define destdir /lib/modules/%{kernel_build}/extra

Summary: TSE Driver for the Kontron TSNIC
Name: tsnic-tse
Packager: Kontron
Vendor: Kontron

Version: %VERSION%
Release: %SNAPSHOT%+%{kernel_release}%{?dist}
License: GPL
Source: %SRC_PACKAGE_NAME%.tar.gz

Requires: kernel-rt-core = %{kernel_version}
BuildRequires: kernel-rt-devel = %{kernel_version}

# Disable the building of the debug package(s).
%define debug_package %{nil}

%description
Driver for tsnic Triple Speed Ethernet (TSE) MAC

%prep
%autosetup -n %SRC_PACKAGE_NAME%

%build
make -C /usr/src/kernels/%{kernel_build}/ M=$PWD modules

%install
make -C /usr/src/kernels/%{kernel_build} INSTALL_MOD_PATH=%{buildroot} M=$PWD modules_install
install -D tsnic-tse.conf %{buildroot}/etc/dracut.conf.d/tsnic-tse.conf

%clean
rm -rf %{buildroot}

%post
/sbin/depmod -a %{kernel_build}

%files
%defattr(-,root,root)
%{destdir}/tsnic-tse.ko
/etc/dracut.conf.d/tsnic-tse.conf
