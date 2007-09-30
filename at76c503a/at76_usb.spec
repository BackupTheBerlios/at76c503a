%define KERNELVER %(rpm -q --qf "%%{version}-%%{release}" kernel-devel)
%define PACKVER 0.17
# RPM Release number of this version
%define TNREL 1

# Real stuff
Summary:   Linux driver for Atmel AT76C503/505/505A based USB WLAN adapters
Name:      at76_usb
Version:   %{PACKVER}
Release:   tn%{TNREL}
License:   GPL
Group:     System Environment/Kernel 
Packager:  Tim Niemueller <tim@niemueller.de>
Source:    %{name}-%{version}.tar.gz
URL:       http://at76c503a.berlios.de/
Prefix:	   %{_prefix}
BuildRequires: kernel = %{KERNELVER}, kernel-devel = %{KERNELVER}
BuildRoot: %{_tmppath}/%{name}-%{version}

%description
This is a Linux driver for the Atmel based USB WLAN adapters.  Unlike
the driver developed by Atmel (atmelwlandriver), this driver is intended
to be accepted into the kernel, so it aims to be bloat-free, streamlined
and modern.

%prep
%setup -n %{name}-%{version}

#if [ ! -e /usr/src/linux-2.4/.config ]; then
#  echo "You need to run 'make menuconfig' once in /usr/src/linux-2.4"
#  echo "Then exit immediately and save the configuration. The "
#  echo "config file is needed to compile hostap. Thanks."
#  exit 1
#fi

%build
make KERNEL_PATH=/lib/modules/%{KERNELVER}/build

%install
rm -rf $RPM_BUILD_ROOT

make install INSTALL_MOD_PATH=$RPM_BUILD_ROOT DEPMOD=true \
	KERNEL_PATH=/lib/modules/%{KERNELVER}/build

%clean 
rm -rf $RPM_BUILD_ROOT

%post
/sbin/depmod -a

%postun
/sbin/depmod -a

%files
%defattr(-,root,root)
/lib/modules/%{KERNELVER}/kernel/drivers/net/wireless/at76_usb.ko

%changelog
* Mon Jul 28 2003 Tim Niemueller <tim@niemueller.de>
- Initial rpm release
- Wrote spec file and compiled for current Red Hat 9 kernel

