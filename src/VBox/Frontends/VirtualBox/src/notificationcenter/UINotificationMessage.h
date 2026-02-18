/* $Id: UINotificationMessage.h 113073 2026-02-18 15:54:09Z sergey.dubov@oracle.com $ */
/** @file
 * VBox Qt GUI - Various UINotificationMessage declarations.
 */

/*
 * Copyright (C) 2021-2026 Oracle and/or its affiliates.
 *
 * This file is part of VirtualBox base platform packages, as
 * available from https://www.virtualbox.org.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation, in version 3 of the
 * License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <https://www.gnu.org/licenses>.
 *
 * SPDX-License-Identifier: GPL-3.0-only
 */

#ifndef FEQT_INCLUDED_SRC_notificationcenter_UINotificationMessage_h
#define FEQT_INCLUDED_SRC_notificationcenter_UINotificationMessage_h
#ifndef RT_WITHOUT_PRAGMA_ONCE
# pragma once
#endif

/* Qt includes: */
#include <QUuid>

/* GUI includes: */
#include "UILibraryDefs.h"
#include "UIMediumDefs.h"
#include "UINotificationObject.h"

/* Forward declarations: */
class UINotificationCenter;
class CAppliance;
class CAudioAdapter;
class CCloudMachine;
class CCloudProviderManager;
class CCloudProvider;
class CCloudProfile;
class CConsole;
class CEmulatedUSB;
class CExtPackFile;
class CExtPackManager;
class CGuest;
class CGuestOSType;
class CHost;
class CHostNetworkInterface;
class CKeyboard;
class CMachine;
class CMachineDebugger;
class CMedium;
class CMediumAttachment;
class CMouse;
class CNetworkAdapter;
class CPlatformProperties;
class CSession;
class CSnapshot;
class CStorageController;
class CSystemProperties;
class CVirtualBox;
class CVirtualBoxErrorInfo;
class CVirtualSystemDescription;
class CVirtualSystemDescriptionForm;
class CVRDEServer;
class CVRDEServerInfo;
class CUnattended;
class CUpdateAgent;
#ifdef VBOX_WITH_DRAG_AND_DROP
class CDnDSource;
class CDnDTarget;
#endif

/** UINotificationSimple extension for message functionality. */
class SHARED_LIBRARY_STUFF UINotificationMessage : public UINotificationSimple
{
    Q_OBJECT;

public:

    /** Possible notification types <= coped from MessageType. */
    enum NotificationType
    {
        NotificationType_Warning = 1,
        NotificationType_Error,
        NotificationType_Critical
    };

    /** Returns whether object is done. */
    virtual bool isDone() const RT_OVERRIDE RT_FINAL { return true; }

    /** @name Simple general warnings.
      * @{ */
        /** Notifies about inability to find help file at certain @a strLocation. */
        static void cannotFindHelpFile(const QString &strLocation);

        /** Notifies about inability to open @a strUrl. */
        static void cannotOpenURL(const QString &strUrl);

        /** Reminds about BETA build. */
        static void remindAboutBetaBuild();
        /** Reminds about BETA build. */
        static void remindAboutExperimentalBuild();

#ifdef RT_OS_LINUX
        /** Notifies about wrong USB mounted. */
        static void warnAboutWrongUSBMounted();
#endif

        /** Notifies about invalid encryption password.
          * @param  strPasswordId  Brings password ID. */
        static void warnAboutInvalidEncryptionPassword(const QString &strPasswordId,
                                                       QWidget *pParent);
        /** Notifies about a clipboard error. */
        static void warnAboutClipboardError(const QString &strMsg);

        /** Notifies about unability to save machine settings.
          * @param  strDetails  Brings the notification details. */
        static void warnAboutCannotSaveSettings(const QString strDetails,
                                                QWidget *pParent);

#ifdef VBOX_GUI_WITH_NETWORK_MANAGER
        /** Notifies about update not found. */
        static void showUpdateNotFound();
        /** Notifies about update successful.
          * @param  strVersion  Brings the found version.
          * @param  strLink     Brings the link to found package. */
        static void showUpdateSuccess(const QString &strVersion, const QString &strLink);
        /** Notifies about extension pack needs to be updated.
          * @param  strExtPackName     Brings the package name.
          * @param  strExtPackVersion  Brings the package version.
          * @param  strVBoxVersion     Brings VBox version. */
        static void askUserToDownloadExtensionPack(const QString &strExtPackName,
                                                   const QString &strExtPackVersion,
                                                   const QString &strVBoxVersion);

        /** Notifies about inability to validate guest additions.
          * @param  strUrl  Brings the GA URL.
          * @param  strSrc  Brings the GA source. */
        static void cannotValidateGuestAdditionsSHA256Sum(const QString &strUrl,
                                                          const QString &strSrc);
        /** Notifies about inability to save guest additions.
          * @param  strUrl  Brings the GA URL.
          * @param  strTgt  Brings the GA target location. */
        static void cannotSaveGuestAdditions(const QString &strUrl,
                                             const QString &strTgt);

        /** Notifies about inability to save user manual.
          * @param  strUrl  Brings the User Manual URL.
          * @param  strTgt  Brings the User Manual target location. */
        static void cannotSaveUserManual(const QString &strURL,
                                         const QString &strTgt);
        /** Notifies about user manual downloded.
          * @param  strUrl  Brings the User Manual URL.
          * @param  strTgt  Brings the User Manual target location. */
        static void warnAboutUserManualDownloaded(const QString &strUrl,
                                                  const QString &strTgt);

        /** Notifies about inability to validate guest additions.
          * @param  strUrl  Brings the GA URL.
          * @param  strSrc  Brings the GA source. */
        static void cannotValidateExtentionPackSHA256Sum(const QString &strExtPackName,
                                                         const QString &strFrom,
                                                         const QString &strTo);
        /** Notifies about inability to save extension pack.
          * @param  strExtPackName  Brings the Extension Pack name.
          * @param  strFrom         Brings the Extension Pack source location.
          * @param  strTo           Brings the Extension Pack target location. */
        static void cannotSaveExtensionPack(const QString &strExtPackName,
                                            const QString &strFrom,
                                            const QString &strTo);
#endif /* VBOX_GUI_WITH_NETWORK_MANAGER */
    /** @} */

    /** @name Simple VirtualBox Manager warnings.
      * @{ */
        /** Notifies about inability to create machine folder.
          * @param  strPath  Brings the machine folder path. */
        static void cannotCreateMachineFolder(const QString &strPath,
                                              QWidget *pParent);
        /** Notifies about inability to overwrite machine folder.
          * @param  strPath  Brings the machine folder path. */
        static void cannotOverwriteMachineFolder(const QString &strPath,
                                                 QWidget *pParent);
        /** Notifies about inability to remove machine folder.
          * @param  strPath  Brings the machine folder path. */
        static void cannotRemoveMachineFolder(const QString &strPath,
                                              QWidget *pParent);
        /** Notifies about inability to move machine folder.
          * @param  strPath  Brings the machine folder path. */
        static void cannotMoveMachineFolder(const QString &strPath);

        /** Notifies about inability to register existing machine.
          * @param  streName     Brings the machine name.
          * @param  strLocation  Brings the machine location. */
        static void cannotReregisterExistingMachine(const QString &strName, const QString &strLocation);

        /** Notifies about inability to resolve collision automatically.
          * @param  strCollisionName  Brings the collision VM name.
          * @param  strGroupName      Brings the group name. */
        static void cannotResolveCollisionAutomatically(const QString &strCollisionName, const QString &strGroupName);

        /** Notifies about inability to acquire cloud machine settings.
          * @param  strErrorDetails  Brings the error details. */
        static void cannotAcquireCloudMachineSettings(const QString &strErrorDetails);

        /** Notifies about inability to create medium storage in FAT.
          * @param  strPath  Brings the medium path. */
        static void cannotCreateMediumStorageInFAT(const QString &strPath,
                                                   QWidget *pParent);
        /** Notifies about inability to overwrite medium storage.
          * @param  strPath  Brings the medium path. */
        static void cannotOverwriteMediumStorage(const QString &strPath,
                                                 QWidget *pParent);

        /** Notifies about inability to open license file.
          * @param  strPath  Brings the license file path. */
        static void cannotOpenLicenseFile(const QString &strPath);

        /** Notifies about public key path is empty. */
        static void warnAboutPublicKeyFilePathIsEmpty();
        /** Notifies about public key file doesn't exist.
          * @param  strPath  Brings the path being checked. */
        static void warnAboutPublicKeyFileDoesntExist(const QString &strPath);
        /** Notifies about public key file is of too large size.
          * @param  strPath  Brings the path being checked. */
        static void warnAboutPublicKeyFileIsOfTooLargeSize(const QString &strPath);
        /** Notifies about public key file isn't readable.
          * @param  strPath  Brings the path being checked. */
        static void warnAboutPublicKeyFileIsntReadable(const QString &strPath);

        /** Notifies about DHCP server isn't enabled.
          * @param  strName  Brings the interface name. */
        static void warnAboutDHCPServerIsNotEnabled(const QString &strName);
        /** Notifies about invalid IPv4 address.
          * @param  strName  Brings the interface name. */
        static void warnAboutInvalidIPv4Address(const QString &strName);
        /** Notifies about invalid IPv4 mask.
          * @param  strName  Brings the interface name. */
        static void warnAboutInvalidIPv4Mask(const QString &strName);
        /** Notifies about invalid IPv6 address.
          * @param  strName  Brings the interface name. */
        static void warnAboutInvalidIPv6Address(const QString &strName);
        /** Notifies about invalid IPv6 prefix length.
          * @param  strName  Brings the interface name. */
        static void warnAboutInvalidIPv6PrefixLength(const QString &strName);
        /** Notifies about invalid DHCP server address.
          * @param  strName  Brings the interface name. */
        static void warnAboutInvalidDHCPServerAddress(const QString &strName);
        /** Notifies about invalid DHCP server mask.
          * @param  strName  Brings the interface name. */
        static void warnAboutInvalidDHCPServerMask(const QString &strName);
        /** Notifies about invalid DHCP server lower address.
          * @param  strName  Brings the interface name. */
        static void warnAboutInvalidDHCPServerLowerAddress(const QString &strName);
        /** Notifies about invalid DHCP server upper address.
          * @param  strName  Brings the interface name. */
        static void warnAboutInvalidDHCPServerUpperAddress(const QString &strName);
        /** Notifies about no name specified.
          * @param  strName  Brings the interface name. */
        static void warnAboutNoNameSpecified(const QString &strName);
        /** Notifies about name already busy.
          * @param  strName  Brings the interface name. */
        static void warnAboutNameAlreadyBusy(const QString &strName);
        /** Notifies about no IPv4 prefix specified.
          * @param  strName  Brings the interface name. */
        static void warnAboutNoIPv4PrefixSpecified(const QString &strName);
        /** Notifies about no IPv6 prefix specified.
          * @param  strName  Brings the interface name. */
        static void warnAboutNoIPv6PrefixSpecified(const QString &strName);

        /** Notifies about incorrect port specified. */
        static bool warnAboutIncorrectPort(QWidget *pParent);
        /** Notifies about incorrect address specified. */
        static bool warnAboutIncorrectAddress(QWidget *pParent);
        /** Notifies about empty guest address specified. */
        static bool warnAboutEmptyGuestAddress(QWidget *pParent);
        /** Notifies about name uniqueness requirement. */
        static bool warnAboutNameShouldBeUnique(QWidget *pParent);
        /** Notifies about rules conflict. */
        static bool warnAboutRulesConflict(QWidget *pParent);

        /** Notifies about state change. */
        static void warnAboutStateChange(QWidget *pParent);
    /** @} */

    /** @name Simple Runtime UI warnings.
      * @{ */
        /** Notifies about runtime error.
          * @param  enmType      Brings the notification type.
          * @param  strErrorId   Brings the error ID.
          * @param  strErrorMsg  Brings the error message. */
        static void showRuntimeError(NotificationType enmType,
                                     const QString &strErrorId,
                                     const QString &strErrorMsg);

        /** Notifies about inability to mount image.
          * @param  strMachineName  Brings the machine name.
          * @param  strMediumName   Brings the medium name. */
        static void cannotMountImage(const QString &strMachineName, const QString &strMediumName);

        /** Notifies about inability to send ACPI shutdown. */
        static void cannotSendACPIToMachine();

        /** Reminds about keyboard auto capturing. */
        static void remindAboutAutoCapture();

        /** Reminds about GA not affected. */
        static void remindAboutGuestAdditionsAreNotActive();

        /** Reminds about mouse integration.
          * @param  fSupportsAbsolute  Brings whether mouse supports absolute pointing. */
        static void remindAboutMouseIntegration(bool fSupportsAbsolute);

        /** Reminds about paused VM input. */
        static void remindAboutPausedVMInput();
        /** Revokes message about paused VM input. */
        static void forgetAboutPausedVMInput();

        /** Reminds about wrong color depth.
          * @param  uRealBPP    Brings real bit per pixel value.
          * @param  uWantedBPP  Brings wanted bit per pixel value. */
        static void remindAboutWrongColorDepth(ulong uRealBPP, ulong uWantedBPP);
        /** Revokes message about wrong color depth. */
        static void forgetAboutWrongColorDepth();

        /** Reminds about VBoxSVC unavailable. */
        static void warnAboutVBoxSVCUnavailable();
    /** @} */

    /** @name COM general warnings.
      * @{ */
        /** Notifies about inability to acquire IVirtualBox parameter.
          * @param  comVBox  Brings the object parameter get acquired from. */
        static void cannotAcquireVirtualBoxParameter(const CVirtualBox &comVBox,
                                                     QWidget *pParent = 0);
        /** Notifies about inability to acquire IAppliance parameter.
          * @param  comVBox  Brings the object parameter get acquired from. */
        static void cannotAcquireApplianceParameter(const CAppliance &comAppliance,
                                                    QWidget *pParent);
        /** Notifies about inability to acquire IPlatform parameter.
          * @param  comPlatform  Brings the object parameter get acquired from. */
        static void cannotAcquirePlatformParameter(const CPlatform &comPlatform);
        /** Notifies about inability to acquire IPlatformProperties parameter.
          * @param  comProperties  Brings the object parameter get acquired from. */
        static void cannotAcquirePlatformPropertiesParameter(const CPlatformProperties &comProperties);
        /** Notifies about inability to acquire ISystemProperties parameter.
          * @param  comProperties  Brings the object parameter get acquired from. */
        static void cannotAcquireSystemPropertiesParameter(const CSystemProperties &comProperties);
        /** Notifies about inability to acquire IExtPackManager parameter.
          * @param  comVBox  Brings the object parameter get acquired from. */
        static void cannotAcquireExtensionPackManagerParameter(const CExtPackManager &comEPManager);
        /** Notifies about inability to acquire IExtPack parameter.
          * @param  comPackage  Brings the object parameter get acquired from. */
        static void cannotAcquireExtensionPackParameter(const CExtPack &comPackage);
        /** Notifies about inability to acquire IHost parameter.
          * @param  comHost  Brings the object parameter get acquired from. */
        static void cannotAcquireHostParameter(const CHost &comHost);
        /** Notifies about inability to acquire IStorageController parameter.
          * @param  comStorageController  Brings the object parameter get acquired from. */
        static void cannotAcquireStorageControllerParameter(const CStorageController &comStorageController);
        /** Notifies about inability to change IStorageController parameter.
          * @param  comStorageController  Brings the object parameter being changed for. */
        static void cannotChangeStorageControllerParameter(const CStorageController &comStorageController);
        /** Notifies about inability to acquire IMediumAttachment parameter.
          * @param  comMediumAttachment  Brings the object parameter get acquired from. */
        static void cannotAcquireMediumAttachmentParameter(const CMediumAttachment &comMediumAttachment);
        /** Notifies about inability to acquire IMedium parameter.
          * @param  comMedium  Brings the object parameter get acquired from. */
        static void cannotAcquireMediumParameter(const CMedium &comMedium);
        /** Notifies about inability to acquire ISession parameter.
          * @param  comSession  Brings the object parameter get acquired from. */
        static void cannotAcquireSessionParameter(const CSession &comSession);
        /** Notifies about inability to acquire IMachine parameter.
          * @param  comMachine  Brings the object parameter get acquired from. */
        static void cannotAcquireMachineParameter(const CMachine &comMachine);
        /** Notifies about inability to acquire IMachineDebugger parameter.
          * @param  comMachineDebugger  Brings the object parameter get acquired from. */
        static void cannotAcquireMachineDebuggerParameter(const CMachineDebugger &comMachineDebugger);
        /** Notifies about inability to acquire IGraphicsAdapter parameter.
          * @param  comAdapter  Brings the object parameter get acquired from. */
        static void cannotAcquireGraphicsAdapterParameter(const CGraphicsAdapter &comAdapter);
        /** Notifies about inability to acquire IAudioSettings parameter.
          * @param  comSettings  Brings the object parameter get acquired from. */
        static void cannotAcquireAudioSettingsParameter(const CAudioSettings &comSettings);
        /** Notifies about inability to acquire IAudioAdapter parameter.
          * @param  comAdapter  Brings the object parameter get acquired from. */
        static void cannotAcquireAudioAdapterParameter(const CAudioAdapter &comAdapter);
        /** Notifies about inability to acquire INetworkAdapter parameter.
          * @param  comAdapter  Brings the object parameter get acquired from. */
        static void cannotAcquireNetworkAdapterParameter(const CNetworkAdapter &comAdapter);
        /** Notifies about inability to acquire IConsole parameter.
          * @param  comConsole  Brings the object parameter get acquired from. */
        static void cannotAcquireConsoleParameter(const CConsole &comConsole);
        /** Notifies about inability to acquire IGuest parameter.
          * @param  comGuest  Brings the object parameter get acquired from. */
        static void cannotAcquireGuestParameter(const CGuest &comGuest);
        /** Notifies about inability to acquire IGuestOSType parameter.
          * @param  comGuestOSType  Brings the object parameter get acquired from. */
        static void cannotAcquireGuestOSTypeParameter(const CGuestOSType &comGuestOSType);
        /** Notifies about inability to acquire ISnapshot parameter.
          * @param  comSnapshot  Brings the object parameter get acquired from. */
        static void cannotAcquireSnapshotParameter(const CSnapshot &comSnapshot);
        /** Notifies about inability to acquire IDHCPServer parameter.
          * @param  comServer  Brings the object parameter get acquired from. */
        static void cannotAcquireDHCPServerParameter(const CDHCPServer &comServer);
        /** Notifies about inability to acquire ICloudNetwork parameter.
          * @param  comNetwork  Brings the object parameter get acquired from. */
        static void cannotAcquireCloudNetworkParameter(const CCloudNetwork &comNetwork);
        /** Notifies about inability to acquire IHostNetworkInterface parameter.
          * @param  comInterface  Brings the object parameter get acquired from. */
        static void cannotAcquireHostNetworkInterfaceParameter(const CHostNetworkInterface &comInterface);
        /** Notifies about inability to acquire IHostOnlyNetwork parameter.
          * @param  comNetwork  Brings the object parameter get acquired from. */
        static void cannotAcquireHostOnlyNetworkParameter(const CHostOnlyNetwork &comNetwork);
        /** Notifies about inability to acquire INATNetwork parameter.
          * @param  comNetwork  Brings the object parameter get acquired from. */
        static void cannotAcquireNATNetworkParameter(const CNATNetwork &comNetwork);
        /** Notifies about inability to acquire IDisplay parameter.
          * @param  comDisplay  Brings the object parameter get acquired from. */
        static void cannotAcquireDisplayParameter(const CDisplay &comDisplay);
        /** Notifies about inability to acquire IUpdateAgent parameter.
          * @param  comAgent  Brings the object parameter get acquired from. */
        static void cannotAcquireUpdateAgentParameter(const CUpdateAgent &comAgent);
        /** Notifies about inability to acquire IMouse parameter.
          * @param  comMouse  Brings the object parameter get acquired from. */
        static void cannotAcquireMouseParameter(const CMouse &comMouse);
        /** Notifies about inability to acquire IEmulatedUSB parameter.
          * @param  comDispatcher  Brings the object parameter get acquired from. */
        static void cannotAcquireEmulatedUSBParameter(const CEmulatedUSB &comDispatcher);
        /** Notifies about inability to acquire IRecordingSettings parameter.
          * @param  comSettings  Brings the object parameter get acquired from. */
        static void cannotAcquireRecordingSettingsParameter(const CRecordingSettings &comSettings);
        /** Notifies about inability to acquire IVRDEServer parameter.
          * @param  comServer  Brings the object parameter get acquired from. */
        static void cannotAcquireVRDEServerParameter(const CVRDEServer &comServer);
        /** Notifies about inability to acquire IVRDEServerInfo parameter.
          * @param  comVRDEServerInfo  Brings the object parameter get acquired from. */
        static void cannotAcquireVRDEServerInfoParameter(const CVRDEServerInfo &comVRDEServerInfo);
        /** Notifies about inability to acquire IVirtualSystemDescription parameter.
          * @param  comVsd  Brings the object parameter get acquired from. */
        static void cannotAcquireVirtualSystemDescriptionParameter(const CVirtualSystemDescription &comVsd,
                                                                   QWidget *pParent);
        /** Notifies about inability to acquire IVirtualSystemDescriptionForm parameter.
          * @param  comVsdForm  Brings the object parameter get acquired from. */
        static void cannotAcquireVirtualSystemDescriptionFormParameter(const CVirtualSystemDescriptionForm &comVsdForm,
                                                                       QWidget *pParent);
        /** Notifies about inability to acquire ICloudProviderManager parameter.
          * @param  comCloudProviderManager  Brings the object parameter get acquired from. */
        static void cannotAcquireCloudProviderManagerParameter(const CCloudProviderManager &comCloudProviderManager,
                                                               QWidget *pParent);
        /** Notifies about inability to acquire ICloudProvider parameter.
          * @param  comCloudProvider  Brings the object parameter get acquired from. */
        static void cannotAcquireCloudProviderParameter(const CCloudProvider &comCloudProvider,
                                                        QWidget *pParent);
        /** Notifies about inability to acquire ICloudProfile parameter.
          * @param  comCloudProfile  Brings the object parameter get acquired from. */
        static void cannotAcquireCloudProfileParameter(const CCloudProfile &comCloudProfile,
                                                       QWidget *pParent);
        /** Notifies about inability to acquire ICloudMachine parameter.
          * @param  comCloudMachine  Brings the object parameter get acquired from. */
        static void cannotAcquireCloudMachineParameter(const CCloudMachine &comCloudMachine,
                                                       QWidget *pParent);

        /** Notifies about inability to change IHost parameter.
          * @param  comHost  Brings the object parameter being changed for. */
        static void cannotChangeHostParameter(const CHost &comHost, QWidget *pParent = 0);
        /** Notifies about inability to change ISystemProperties.
          * @param  comProperties  Brings the properties object being changed. */
        static void cannotChangeSystemProperties(const CSystemProperties &comProperties, QWidget *pParent = 0);
        /** Notifies about inability to change IMedium parameter.
          * @param  comMedium  Brings the object parameter being changed for. */
        static void cannotChangeMediumParameter(const CMedium &comMedium);
        /** Notifies about inability to change IMachine parameter.
          * @param  comMachine  Brings the object parameter being changed for. */
        static void cannotChangeMachineParameter(const CMachine &comMachine);
        /** Notifies about inability to change IMachineDebugger parameter.
          * @param  comMachineDebugger  Brings the object parameter being changed for. */
        static void cannotChangeMachineDebuggerParameter(const CMachineDebugger &comMachineDebugger);
        /** Notifies about inability to change IGraphicsAdapter parameter.
          * @param  comAdapter  Brings the object parameter being changed for. */
        static void cannotChangeGraphicsAdapterParameter(const CGraphicsAdapter &comAdapter);
        /** Notifies about inability to change IAudioAdapter parameter.
          * @param  comAdapter  Brings the object parameter being changed for. */
        static void cannotChangeAudioAdapterParameter(const CAudioAdapter &comAdapter);
        /** Notifies about inability to change INetworkAdapter parameter.
          * @param  comAdapter  Brings the object parameter being changed for. */
        static void cannotChangeNetworkAdapterParameter(const CNetworkAdapter &comAdapter);
        /** Notifies about inability to change IDHCPServer parameter.
          * @param  comServer  Brings the object parameter being changed for. */
        static void cannotChangeDHCPServerParameter(const CDHCPServer &comServer);
        /** Notifies about inability to change ICloudNetwork parameter.
          * @param  comNetwork  Brings the object parameter being changed for. */
        static void cannotChangeCloudNetworkParameter(const CCloudNetwork &comNetwork);
        /** Notifies about inability to change IHostNetworkInterface parameter.
          * @param  comInterface  Brings the object parameter being changed for. */
        static void cannotChangeHostNetworkInterfaceParameter(const CHostNetworkInterface &comInterface);
        /** Notifies about inability to change IHostOnlyNetwork parameter.
          * @param  comNetwork  Brings the object parameter being changed for. */
        static void cannotChangeHostOnlyNetworkParameter(const CHostOnlyNetwork &comNetwork);
        /** Notifies about inability to change INATNetwork parameter.
          * @param  comNetwork  Brings the object parameter being changed for. */
        static void cannotChangeNATNetworkParameter(const CNATNetwork &comNetwork);
        /** Notifies about inability to change IDisplay parameter.
          * @param  comDisplay  Brings the object parameter being changed for. */
        static void cannotChangeDisplayParameter(const CDisplay &comDisplay);
        /** Notifies about inability to change ICloudProfile parameter.
          * @param  comProfile  Brings the object parameter being changed for. */
        static void cannotChangeCloudProfileParameter(const CCloudProfile &comProfile);
        /** Notifies about inability to change IUpdateAgent parameter.
          * @param  comAgent  Brings the object parameter being changed for. */
        static void cannotChangeUpdateAgentParameter(const CUpdateAgent &comAgent);
        /** Notifies about inability to change IKeyboard parameter.
          * @param  comKeyboard  Brings the object parameter being changed for. */
        static void cannotChangeKeyboardParameter(const CKeyboard &comKeyboard);
        /** Notifies about inability to change IVirtualSystemDescription parameter.
          * @param  comVsd  Brings the object parameter being changed for. */
        static void cannotChangeVirtualSystemDescriptionParameter(const CVirtualSystemDescription &comVsd,
                                                                  QWidget *pParent);

        /** Notifies about inability to enumerate host USB devices.
          * @param  comHost  Brings the host devices enumerated for. */
        static void cannotEnumerateHostUSBDevices(const CHost &comHost);
        /** Notifies about inability to access USB subsystem.
          * @param  comObject  Brings the machine USB subsystem accessed for. */
        static void cannotAccessUSBSubsystem(const CMachine &comMachine, QWidget *pParent);
        /** Notifies about inability to open medium.
          * @param  comVBox      Brings common VBox object trying to open medium.
          * @param  strLocation  Brings the medium location. */
        static void cannotOpenMedium(const CVirtualBox &comVBox, const QString &strLocation,
                                     QWidget *pParent = 0);

        /** Notifies about inability to pause machine.
          * @param  comConsole  Brings console trying to pause machine. */
        static void cannotPauseMachine(const CConsole &comConsole);
        /** Notifies about inability to resume machine.
          * @param  comConsole  Brings console trying to resume machine. */
        static void cannotResumeMachine(const CConsole &comConsole);
        /** Notifies about inability to ACPI shutdown machine.
          * @param  comConsole  Brings console trying to shutdown machine. */
        static void cannotACPIShutdownMachine(const CConsole &comConsole);
        /** Notifies about inability to reset machine.
          * @param  comConsole  Brings console trying to reset machine. */
        static void cannotResetMachine(const CConsole &comConsole);

        /** Notifies about inability to set machine groups.
          * @param  comMachine  Brings the machine to set groups for. */
        static void cannotSetGroups(const CMachine &comMachine);
    /** @} */

    /** @name COM VirtualBox Manager warnings.
      * @{ */
        /** Notifies about inability to create appliance.
          * @param  comVBox  Brings common VBox object trying to create appliance. */
        static void cannotCreateAppliance(const CVirtualBox &comVBox, QWidget *pParent);
        /** Notifies about inability to register machine.
          * @param  comVBox  Brings common VBox object trying to register machine.
          * @param  strName  Brings the name of VM being registered. */
        static void cannotRegisterMachine(const CVirtualBox &comVBox, const QString &strName, QWidget *pParent = 0);
        /** Notifies about inability to create machine.
          * @param  comVBox  Brings common VBox object trying to create machine. */
        static void cannotCreateMachine(const CVirtualBox &comVBox, QWidget *pParent);
        /** Notifies about inability to find machine by ID.
          * @param  comVBox     Brings common VBox object trying to find machine.
          * @param  uMachineId  Brings the machine ID. */
        static void cannotFindMachineById(const CVirtualBox &comVBox,
                                          const QUuid &uMachineId,
                                          QWidget *pParent);
        /** Notifies about inability to open machine.
          * @param  comVBox      Brings common VBox object trying to open machine.
          * @param  strLocation  Brings the machine location. */
        static void cannotOpenMachine(const CVirtualBox &comVBox, const QString &strLocation);
        /** Notifies about inability to create medium storage.
          * @param  comVBox  Brings common VBox object trying to create medium storage.
          * @param  strPath  Brings the medium path. */
        static void cannotCreateMediumStorage(const CVirtualBox &comVBox,
                                              const QString &strPath,
                                              QWidget *pParent);
        /** Notifies about inability to get ext pack manager.
          * @param  comVBox      Brings common VBox object trying to open machine. */
        static void cannotGetExtensionPackManager(const CVirtualBox &comVBox);

        /** Notifies about inability to create VFS explorer.
          * @param  comAppliance  Brings appliance trying to create VFS explorer. */
        static void cannotCreateVfsExplorer(const CAppliance &comAppliance, QWidget *pParent);
        /** Notifies about inability to add disk scryption password.
          * @param  comAppliance  Brings appliance trying to add disk scryption password. */
        static void cannotAddDiskEncryptionPassword(const CAppliance &comAppliance, QWidget *pParent);
        /** Notifies about inability to interpret appliance.
          * @param  comAppliance  Brings appliance we are trying to interpret. */
        static void cannotInterpretAppliance(const CAppliance &comAppliance, QWidget *pParent);
        /** Notifies about inability to create VSD.
          * @param  comAppliance  Brings appliance trying to create VSD. */
        static void cannotCreateVirtualSystemDescription(const CAppliance &comAppliance, QWidget *pParent);

        /** Notifies about inability to open extension pack.
          * @param  comExtPackManager  Brings extension pack manager trying to open extension pack.
          * @param  strFilename        Brings extension pack file name. */
        static void cannotOpenExtPack(const CExtPackManager &comExtPackManager, const QString &strFilename);
        /** Notifies about inability to read extpack file.
          * @param  comExtPackFile  Brings extension pack manager trying to open extension pack.
          * @param  strFilename     Brings extension pack file name. */
        static void cannotReadExtPack(const CExtPackFile &comExtPackFile, const QString &strFilename);

        /** Notifies about inability to find cloud network.
          * @param  comVBox         Brings common VBox object being search through.
          * @param  strNetworkName  Brings network name. */
        static void cannotFindCloudNetwork(const CVirtualBox &comVBox, const QString &strNetworkName);
        /** Notifies about inability to find host network interface.
          * @param  comHost           Brings the host being search through.
          * @param  strInterfaceName  Brings interface name. */
        static void cannotFindHostNetworkInterface(const CHost &comHost, const QString &strInterfaceName);
        /** Notifies about inability to find host only network.
          * @param  comVBox         Brings the common VBox object being search through.
          * @param  strNetworkName  Brings interface name. */
        static void cannotFindHostOnlyNetwork(const CVirtualBox &comVBox, const QString &strNetworkName);
        /** Notifies about inability to find NAT network.
          * @param  comVBox         Brings common VBox object being search through.
          * @param  strNetworkName  Brings network name. */
        static void cannotFindNATNetwork(const CVirtualBox &comVBox, const QString &strNetworkName);
        /** Notifies about inability to create DHCP server.
          * @param  comVBox           Brings common VBox object trying to create DHCP server.
          * @param  strInterfaceName  Brings the interface name. */
        static void cannotCreateDHCPServer(const CVirtualBox &comVBox, const QString &strInterfaceName);
        /** Notifies about inability to remove DHCP server.
          * @param  comVBox           Brings common VBox object trying to remove DHCP server.
          * @param  strInterfaceName  Brings the interface name. */
        static void cannotRemoveDHCPServer(const CVirtualBox &comVBox, const QString &strInterfaceName);
        /** Notifies about inability to create cloud network.
          * @param  comVBox  Brings common VBox object trying to create cloud network. */
        static void cannotCreateCloudNetwork(const CVirtualBox &comVBox);
        /** Notifies about inability to remove cloud network.
          * @param  comVBox         Brings common VBox object trying to remove cloud network.
          * @param  strNetworkName  Brings the network name. */
        static void cannotRemoveCloudNetwork(const CVirtualBox &comVBox, const QString &strNetworkName);
        /** Notifies about inability to create host only network.
          * @param  comVBox  Brings common VBox object trying to create host only network. */
        static void cannotCreateHostOnlyNetwork(const CVirtualBox &comVBox);
        /** Notifies about inability to remove host only network.
          * @param  comVBox         Brings common VBox object trying to remove host only network.
          * @param  strNetworkName  Brings the network name. */
        static void cannotRemoveHostOnlyNetwork(const CVirtualBox &comVBox, const QString &strNetworkName);
        /** Notifies about inability to create NAT network.
          * @param  comVBox  Brings common VBox object trying to create NAT network. */
        static void cannotCreateNATNetwork(const CVirtualBox &comVBox);
        /** Notifies about inability to remove NAT network.
          * @param  comVBox         Brings common VBox object trying to remove NAT network.
          * @param  strNetworkName  Brings the network name. */
        static void cannotRemoveNATNetwork(const CVirtualBox &comVBox, const QString &strNetworkName);

        /** Notifies about inability to create cloud profile.
          * @param  comProvider  Brings the provider profile being created for. */
        static void cannotCreateCloudProfile(const CCloudProvider &comProvider);
        /** Notifies about inability to remove cloud profile.
          * @param  comProvider  Brings the provider profile being removed from. */
        static void cannotRemoveCloudProfile(const CCloudProfile &comProfile);
        /** Notifies about inability to save cloud profiles.
          * @param  comProvider  Brings the provider profiles being saved for. */
        static void cannotSaveCloudProfiles(const CCloudProvider &comProvider);
        /** Notifies about inability to import cloud profiles.
          * @param  comProvider  Brings the provider profiles being imported for. */
        static void cannotImportCloudProfiles(const CCloudProvider &comProvider);
        /** Notifies about inability to refresh cloud machine.
          * @param  comMachine  Brings the machine being refreshed. */
        static void cannotRefreshCloudMachine(const CCloudMachine &comMachine);
        /** Notifies about inability to refresh cloud machine.
          * @param  comProgress  Brings the progress of machine being refreshed. */
        static void cannotRefreshCloudMachine(const CProgress &comProgress);
        /** Notifies about inability to create cloud client.
          * @param  comProfile  Brings the profile client being created for. */
        static void cannotCreateCloudClient(const CCloudProfile &comProfile, QWidget *pParent);

        /** Notifies about inability to open machine.
          * @param  comMedium  Brings the medium being closed. */
        static void cannotCloseMedium(const CMedium &comMedium);

        /** Notifies about inability to discard saved state.
          * @param  comMachine  Brings the collision VM name. */
        static void cannotDiscardSavedState(const CMachine &comMachine);
        /** Notifies about inability to remove machine.
          * @param  comMachine  Brings machine being removed. */
        static void cannotRemoveMachine(const CMachine &comMachine, QWidget *pParent = 0);
        /** Notifies about inability to export appliance.
          * @param  comMachine  Brings machine trying to export appliance. */
        static void cannotExportMachine(const CMachine &comMachine, QWidget *pParent);
        /** Notifies about inability to attach device.
          * @param  comMachine  Brings machine trying to attach device. */
        static void cannotAttachDevice(const CMachine &comMachine,
                                       UIMediumDeviceType enmType,
                                       const QString &strLocation,
                                       const StorageSlot &storageSlot,
                                       QWidget *pParent = 0);
        /** Notifies about inability to attach device.
          * @param  comMachine  Brings machine trying to attach device. */
        static void cannotDetachDevice(const CMachine &comMachine,
                                       UIMediumDeviceType enmType,
                                       const QString &strLocation,
                                       const StorageSlot &storageSlot,
                                       QWidget *pParent = 0);

        /** Notifies about inability to find snapshot by ID.
          * @param  comMachine  Brings the machine being searched for particular snapshot.
          * @param  uId         Brings the required snapshot ID. */
        static void cannotFindSnapshotById(const CMachine &comMachine, const QUuid &uId);
        /** Notifies about inability to find snapshot by name.
          * @param  comMachine  Brings the machine being searched for particular snapshot.
          * @param  strName     Brings the required snapshot name. */
        static void cannotFindSnapshotByName(const CMachine &comMachine, const QString &strName, QWidget *pParent);
        /** Notifies about inability to change snapshot.
          * @param  comSnapshot      Brings the snapshot being changed.
          * @param  strSnapshotName  Brings snapshot name.
          * @param  strMachineName   Brings machine name. */
        static void cannotChangeSnapshot(const CSnapshot &comSnapshot, const QString &strSnapshotName, const QString &strMachineName);

        /** Notifies about inability to run unattended guest install.
          * @param  comUnattended  Brings the unattended being running guest install. */
        static void cannotRunUnattendedGuestInstall(const CUnattended &comUnattended);
    /** @} */

    /** @name COM Runtime UI warnings.
      * @{ */
        /** Notifies about inability to start machine under the certain @a strName.
          * @param  comConsole  Brings console for machine being started. */
        static void cannotStartMachine(const CConsole &comConsole, const QString &strName);
        /** Notifies about inability to start machine under the certain @a strName.
          * @param  comProgress  Brings progress for machine being started. */
        static void cannotStartMachine(const CProgress &comProgress, const QString &strName);

        /** Notifies about inability to add disk scryption password.
          * @param  comConsole  Brings console trying to add disk scryption password. */
        static void cannotAddDiskEncryptionPassword(const CConsole &comConsole);

        /** Notifies about inability to attach USB device.
          * @param  comConsole  Brings console USB device belongs to.
          * @param  strDevice   Brings the device name. */
        static void cannotAttachUSBDevice(const CConsole &comConsole, const QString &strDevice);
        /** Notifies about inability to attach USB device.
          * @param  comErrorInfo    Brings info about error happened.
          * @param  strDevice       Brings the device name.
          * @param  strMachineName  Brings the machine name. */
        static void cannotAttachUSBDevice(const CVirtualBoxErrorInfo &comErrorInfo,
                                          const QString &strDevice, const QString &strMachineName);
        /** Notifies about inability to detach USB device.
          * @param  comConsole  Brings console USB device belongs to.
          * @param  strDevice   Brings the device name. */
        static void cannotDetachUSBDevice(const CConsole &comConsole, const QString &strDevice);
        /** Notifies about inability to detach USB device.
          * @param  comErrorInfo    Brings info about error happened.
          * @param  strDevice       Brings the device name.
          * @param  strMachineName  Brings the machine name. */
        static void cannotDetachUSBDevice(const CVirtualBoxErrorInfo &comErrorInfo,
                                          const QString &strDevice, const QString &strMachineName);

        /** Notifies about inability to attach webcam.
          * @param  comDispatcher   Brings emulated USB dispatcher webcam being attached to.
          * @param  strWebCamName   Brings the webcam name.
          * @param  strMachineName  Brings the machine name. */
        static void cannotAttachWebCam(const CEmulatedUSB &comDispatcher,
                                       const QString &strWebCamName, const QString &strMachineName);
        /** Notifies about inability to detach webcam.
          * @param  comDispatcher   Brings emulated USB dispatcher webcam being detached from.
          * @param  strWebCamName   Brings the webcam name.
          * @param  strMachineName  Brings the machine name. */
        static void cannotDetachWebCam(const CEmulatedUSB &comDispatcher,
                                       const QString &strWebCamName, const QString &strMachineName);

        /** Notifies about inability to save machine settings.
          * @param  comMachine  Brings the machine trying to save settings. */
        static void cannotSaveMachineSettings(const CMachine &comMachine, QWidget *pParent = 0);

        /** Notifies about inability to toggle audio input.
          * @param  comAdapter      Brings the adapter input being toggled for.
          * @param  strMachineName  Brings the machine name.
          * @param  fEnable         Brings whether adapter input is enabled or not. */
        static void cannotToggleAudioInput(const CAudioAdapter &comAdapter,
                                           const QString &strMachineName, bool fEnable);
        /** Notifies about inability to toggle audio output.
          * @param  comAdapter      Brings the adapter output being toggled for.
          * @param  strMachineName  Brings the machine name.
          * @param  fEnable         Brings whether adapter output is enabled or not. */
        static void cannotToggleAudioOutput(const CAudioAdapter &comAdapter,
                                            const QString &strMachineName, bool fEnable);

        /** Notifies about inability to toggle network cable.
          * @param  comAdapter      Brings the adapter network cable being toggled for.
          * @param  strMachineName  Brings the machine name.
          * @param  fConnect        Brings whether network cable is connected or not. */
        static void cannotToggleNetworkCable(const CNetworkAdapter &comAdapter,
                                             const QString &strMachineName, bool fConnect);

        /** Notifies about inability to toggle recording.
          * @param  comRecording    Brings the recording settings being toggled for.
          * @param  strMachineName  Brings the machine name.
          * @param  fEnable         Brings whether recording is enabled or not. */
        static void cannotToggleRecording(const CRecordingSettings &comRecording, const QString &strMachineName, bool fEnable);

        /** Notifies about inability to toggle VRDE server.
          * @param  comServer       Brings the server being toggled.
          * @param  strMachineName  Brings the machine name.
          * @param  fEnable         Brings whether server is enabled or not. */
        static void cannotToggleVRDEServer(const CVRDEServer &comServer,
                                           const QString &strMachineName, bool fEnable);

#ifdef VBOX_WITH_DRAG_AND_DROP
        /** Notifies about inability to drop data to guest.
          * @param  comDndTarget  Brings the data being dropped. */
        static void cannotDropDataToGuest(const CDnDTarget &comDndTarget);
        /** Notifies about inability to drop data to guest.
          * @param  comProgress  Brings the drop-progress being executed. */
        static void cannotDropDataToGuest(const CProgress &comProgress);

        /** Notifies about inability to drop data to host.
          * @param  comDnDSource  Brings the data being dropped. */
        static void cannotDropDataToHost(const CDnDSource &comDnDSource);
        /** Notifies about inability to drop data to host.
          * @param  comProgress  Brings the drop-progress being executed. */
        static void cannotDropDataToHost(const CProgress &comProgress);
#endif /* VBOX_WITH_DRAG_AND_DROP */
    /** @} */

protected:

    /** Constructs message notification-object.
      * @param  strName          Brings the message name.
      * @param  strDetails       Brings the message details.
      * @param  strInternalName  Brings the message internal name.
      * @param  strHelpKeyword   Brings the message help keyword. */
    UINotificationMessage(const QString &strName,
                          const QString &strDetails,
                          const QString &strInternalName,
                          const QString &strHelpKeyword);
    /** Destructs message notification-object. */
    virtual ~UINotificationMessage() RT_OVERRIDE RT_FINAL;

private:

    /** Creates message.
      * @param  pParent          Brings the local notification-center reference.
      * @param  strName          Brings the message name.
      * @param  strDetails       Brings the message details.
      * @param  strInternalName  Brings the message internal name.
      * @param  strHelpKeyword   Brings the message help keyword. */
    static void createMessageInt(UINotificationCenter *pParent,
                                 const QString &strName,
                                 const QString &strDetails,
                                 const QString &strInternalName,
                                 const QString &strHelpKeyword);
    /** Creates blocking message.
      * @param  pParent          Brings the local notification-center reference.
      * @param  strName          Brings the message name.
      * @param  strDetails       Brings the message details.
      * @param  strInternalName  Brings the message internal name.
      * @param  strHelpKeyword   Brings the message help keyword. */
    static void createBlockingMessageInt(UINotificationCenter *pParent,
                                         const QString &strName,
                                         const QString &strDetails,
                                         const QString &strInternalName,
                                         const QString &strHelpKeyword);

    /** Creates message.
      * @param  strName     Brings the message name.
      * @param  strDetails  Brings the message details.
      * @param  pParent     Brings the parent reference. */
    static void createMessage(const QString &strName,
                              const QString &strDetails,
                              QWidget *pParent = 0);
    /** Creates message.
      * @param  strName          Brings the message name.
      * @param  strDetails       Brings the message details.
      * @param  strInternalName  Brings the message internal name.
      * @param  strHelpKeyword   Brings the message help keyword.
      * @param  pParent          Brings the local notification-center reference. */
    static void createMessage(const QString &strName,
                              const QString &strDetails,
                              const QString &strInternalName,
                              const QString &strHelpKeyword = QString(),
                              QWidget *pParent = 0);
    /** Creates blocking message.
      * @param  strName     Brings the message name.
      * @param  strDetails  Brings the message details.
      * @param  pParent     Brings the parent reference. */
    static void createBlockingMessage(const QString &strName,
                                      const QString &strDetails,
                                      QWidget *pParent = 0);
    /** Creates blocking message.
      * @param  strName          Brings the message name.
      * @param  strDetails       Brings the message details.
      * @param  strInternalName  Brings the message internal name.
      * @param  strHelpKeyword   Brings the message help keyword.
      * @param  pParent          Brings the local notification-center reference. */
    static void createBlockingMessage(const QString &strName,
                                      const QString &strDetails,
                                      const QString &strInternalName,
                                      const QString &strHelpKeyword = QString(),
                                      QWidget *pParent = 0);

    /** Destroys message.
      * @param  strInternalName  Brings the message internal name.
      * @param  pParent          Brings the local notification-center reference. */
    static void destroyMessage(const QString &strInternalName,
                               UINotificationCenter *pParent = 0);

    /** Holds the IDs of messages registered. */
    static QMap<QString, QUuid>  m_messages;
};

#endif /* !FEQT_INCLUDED_SRC_notificationcenter_UINotificationMessage_h */
