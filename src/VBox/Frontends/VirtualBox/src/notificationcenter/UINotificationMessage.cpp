/* $Id: UINotificationMessage.cpp 113265 2026-03-05 08:50:41Z sergey.dubov@oracle.com $ */
/** @file
 * VBox Qt GUI - Various UINotificationMessage implementations.
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

/* Qt includes: */
#include <QApplication>
#include <QFileInfo>

/* GUI includes: */
#include "UIConverter.h"
#include "UIErrorString.h"
#include "UIExtraDataManager.h"
#include "UIHostComboEditor.h"
#include "UILoggingDefs.h"
#include "UINotificationCenter.h"
#include "UINotificationMessage.h"
#include "UITranslator.h"

/* COM includes: */
#include "CAudioAdapter.h"
#include "CAudioSettings.h"
#include "CCloudNetwork.h"
#include "CCloudProfile.h"
#include "CCloudProvider.h"
#include "CCloudProviderManager.h"
#include "CConsole.h"
#include "CDHCPServer.h"
#include "CDisplay.h"
#include "CEmulatedUSB.h"
#include "CExtPack.h"
#include "CGraphicsAdapter.h"
#include "CGuestOSType.h"
#include "CHostNetworkInterface.h"
#include "CHostOnlyNetwork.h"
#include "CKeyboard.h"
#include "CMachineDebugger.h"
#include "CMediumAttachment.h"
#include "CMouse.h"
#include "CNATNetwork.h"
#include "CNetworkAdapter.h"
#include "CPlatform.h"
#include "CPlatformProperties.h"
#include "CRecordingSettings.h"
#include "CStorageController.h"
#include "CSystemProperties.h"
#include "CUnattended.h"
#include "CUpdateAgent.h"
#include "CVirtualBox.h"
#include "CVRDEServer.h"
#include "CVRDEServerInfo.h"
#ifdef VBOX_WITH_DRAG_AND_DROP
# include "CDnDSource.h"
# include "CDnDTarget.h"
#endif


/*********************************************************************************************************************************
*   Class UINotificationMessage implementation.                                                                                  *
*********************************************************************************************************************************/

/* static */
QMap<QString, QUuid> UINotificationMessage::m_messages = QMap<QString, QUuid>();

/* static */
void UINotificationMessage::cannotFindHelpFile(const QString &strLocation)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't find help file ..."),
        QApplication::translate("UIMessageCenter", "Failed to find the following help file: <b>%1</b>")
                                                   .arg(strLocation));
}

/* static */
void UINotificationMessage::cannotOpenURL(const QString &strUrl)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't open URL ..."),
        QApplication::translate("UIMessageCenter", "Failed to open <tt>%1</tt>. "
                                                   "Make sure your desktop environment can properly handle URLs of this type.")
                                                   .arg(strUrl));
}

/* static */
void UINotificationMessage::remindAboutBetaBuild()
{
    createMessage(
        QApplication::translate("UIMessageCenter", "BETA build warning!"),
        QApplication::translate("UIMessageCenter", "You are running a prerelease version of VirtualBox. "
                                                   "This version is not suitable for production use."));
}

/* static */
void UINotificationMessage::remindAboutExperimentalBuild()
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Experimental build warning!"),
        QApplication::translate("UIMessageCenter", "You are running an EXPERIMENTAL build of VirtualBox. "
                                                   "This version is not suitable for production use."));
}

#ifdef RT_OS_LINUX
/* static */
void UINotificationMessage::warnAboutWrongUSBMounted()
{
    createMessage(QApplication::translate("UIMessageCenter", "USBFS filesystem failure ..."),
                  QApplication::translate("UIMessageCenter", "You seem to have the USBFS filesystem mounted at "
                                                             "/sys/bus/usb/drivers.  We strongly recommend that you change this, "
                                                             "as it is a severe mis-configuration of your system which could "
                                                             "cause USB devices to fail in unexpected ways."),
                  "warnAboutWrongUSBMounted");
}
#endif /* RT_OS_LINUX */

/* static */
void UINotificationMessage::warnAboutInvalidEncryptionPassword(const QString &strPasswordId,
                                                               QWidget *pParent)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Invalid Password ..."),
        QApplication::translate("UIMessageCenter", "Encryption password for <nobr>ID = '%1'</nobr> is invalid.")
                                                   .arg(strPasswordId),
        pParent);
}

/* static */
void UINotificationMessage::warnAboutClipboardError(const QString &strMsg)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Shared clipboard error ..."),
        strMsg.toUtf8().constData());
}

/* static */
void UINotificationMessage::warnAboutCannotSaveSettings(const QString strDetails,
                                                        QWidget *pParent)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Failed to save the settings."),
        strDetails.toUtf8().constData(),
        pParent);
}

#ifdef VBOX_GUI_WITH_NETWORK_MANAGER
/* static */
void UINotificationMessage::showUpdateNotFound()
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Nothing to update ..."),
        QApplication::translate("UIMessageCenter", "You are already running the most recent version of VirtualBox."));
}

/* static */
void UINotificationMessage::showUpdateSuccess(const QString &strVersion, const QString &strLink)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "New version found ..."),
        QApplication::translate("UIMessageCenter", "<p>A new version of VirtualBox has been released! Version <b>%1</b> is available "
                                                   "at <a href=\"https://www.virtualbox.org/\">virtualbox.org</a>.</p>"
                                                   "<p>You can download this version using the link:</p>"
                                                   "<p><a href=%2>%3</a></p>").arg(strVersion, strLink, strLink));
}

/* static */
void UINotificationMessage::askUserToDownloadExtensionPack(const QString &strExtPackName,
                                                           const QString &strExtPackVersion,
                                                           const QString &strVBoxVersion)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Update is required ..."),
        QApplication::translate("UIMessageCenter", "<p>You have version %1 of the <b><nobr>%2</nobr></b> installed.</p>"
                                                   "<p>You should download and install version %3 of this extension pack from "
                                                   "Oracle!</p>").arg(strExtPackVersion, strExtPackName, strVBoxVersion));
}

/* static */
void UINotificationMessage::cannotValidateGuestAdditionsSHA256Sum(const QString &strUrl,
                                                                  const QString &strSrc)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Unable to validate guest additions image ..."),
        QApplication::translate("UIMessageCenter", "<p>The <b>VirtualBox Guest Additions</b> disk image file has been "
                                                   "successfully downloaded from <nobr><a href=\"%1\">%1</a></nobr> and saved "
                                                   "locally as <nobr><b>%2</b>, </nobr>but the SHA-256 checksum verification "
                                                   "failed.</p><p>Please do the download, installation and verification "
                                                   "manually.</p>").arg(strUrl, strSrc));
}

/* static */
void UINotificationMessage::cannotSaveGuestAdditions(const QString &strUrl,
                                                     const QString &strTgt)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Unable to save guest additions image ..."),
        QApplication::translate("UIMessageCenter", "<p>The <b>VirtualBox Guest Additions</b> disk image file has been "
                                                   "successfully downloaded from <nobr><a href=\"%1\">%1</a></nobr> but "
                                                   "can't be saved locally as <nobr><b>%2</b>.</nobr></p><p>Please choose "
                                                   "another location for that file.</p>").arg(strUrl, strTgt));
}

/* static */
void UINotificationMessage::cannotValidateExtentionPackSHA256Sum(const QString &strExtPackName,
                                                                 const QString &strFrom,
                                                                 const QString &strTo)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Unable to validate extension pack ..."),
        QApplication::translate("UIMessageCenter", "<p>The <b><nobr>%1</nobr></b> has been successfully downloaded "
                                                   "from <nobr><a href=\"%2\">%2</a></nobr> and saved locally as "
                                                   "<nobr><b>%3</b>, </nobr>but the SHA-256 checksum verification failed.</p>"
                                                   "<p>Please do the download, installation and verification manually.</p>")
                                                   .arg(strExtPackName, strFrom, strTo));
}

/* static */
void UINotificationMessage::cannotSaveExtensionPack(const QString &strExtPackName,
                                                    const QString &strFrom,
                                                    const QString &strTo)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Unable to save extension pack file ..."),
        QApplication::translate("UIMessageCenter", "<p>The <b><nobr>%1</nobr></b> has been successfully downloaded from "
                                                   "<nobr><a href=\"%2\">%2</a></nobr> but can't be saved locally as "
                                                   "<nobr><b>%3</b>.</nobr></p><p>Please choose another location for that "
                                                   "file.</p>").arg(strExtPackName, strFrom, strTo));
}
#endif /* VBOX_GUI_WITH_NETWORK_MANAGER */

/* static */
void UINotificationMessage::cannotCreateMachineFolder(const QString &strPath,
                                                      QWidget *pParent)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't create machine folder ..."),
        QApplication::translate("UIMessageCenter", "Failed to create machine folder at <nobr><b>%1</b></nobr>.")
                                                   .arg(strPath),
        pParent);
}

/* static */
void UINotificationMessage::cannotOverwriteMachineFolder(const QString &strPath,
                                                         QWidget *pParent)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't overwrite machine folder ..."),
        QApplication::translate("UIMessageCenter", "Failed to overwrite machine folder at <nobr><b>%1</b></nobr>.")
                                                   .arg(strPath),
        pParent);
}

/* static */
void UINotificationMessage::cannotRemoveMachineFolder(const QString &strPath,
                                                      QWidget *pParent)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't remove machine folder ..."),
        QApplication::translate("UIMessageCenter", "Failed to remove machine folder at <nobr><b>%1</b></nobr>.")
                                                   .arg(strPath),
        pParent);
}

/* static */
void UINotificationMessage::cannotMoveMachineFolder(const QString &strPath)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't move machine folder ..."),
        QApplication::translate("UIMessageCenter", "Failed to move machine folder to <nobr><b>%1</b></nobr>.  "
                                                   "A file with the same name already exists!")
                                                   .arg(strPath));
}

/* static */
void UINotificationMessage::cannotReregisterExistingMachine(const QString &strName, const QString &strLocation)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't add machine ..."),
        QApplication::translate("UIMessageCenter", "Failed to add virtual machine <b>%1</b> located in <i>%2</i> because its "
                                                   "already present.")
                                                   .arg(strName, strLocation));
}

/* static */
void UINotificationMessage::cannotResolveCollisionAutomatically(const QString &strCollisionName, const QString &strGroupName)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't resolve collision ..."),
        QApplication::translate("UIMessageCenter", "<p>You are trying to move machine <nobr><b>%1</b></nobr> to group "
                                                   "<nobr><b>%2</b></nobr> which already have another item with the same "
                                                   "name.</p><p>Please resolve this name conflict and try again.</p>")
                                                   .arg(strCollisionName, strGroupName));
}

/* static */
void UINotificationMessage::cannotAcquireCloudMachineSettings(const QString &strErrorDetails)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Cloud failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to acquire cloud machine settings.") +
        strErrorDetails);
}

/* static */
void UINotificationMessage::cannotCreateMediumStorageInFAT(const QString &strPath,
                                                           QWidget *pParent)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't create medium ..."),
        QApplication::translate("UIMessageCenter", "Failed to create medium storage at <nobr><b>%1</b></nobr>.")
                                                   .arg(strPath),
        pParent);
}

/* static */
void UINotificationMessage::cannotOverwriteMediumStorage(const QString &strPath,
                                                         QWidget *pParent)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't overwrite medium ..."),
        QApplication::translate("UIMessageCenter", "Failed to overwrite medium storage at <nobr><b>%1</b></nobr>.")
                                                   .arg(strPath),
        pParent);
}

/* static */
void UINotificationMessage::cannotOpenLicenseFile(const QString &strPath)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't open license file ..."),
        QApplication::translate("UIMessageCenter", "Failed to open the license file <nobr><b>%1</b></nobr>. Check file "
                                                   "permissions.").arg(strPath));
}

/* static */
void UINotificationMessage::warnAboutPublicKeyFilePathIsEmpty()
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Public key missing ..."),
        QApplication::translate("UIMessageCenter", "Public key file path is empty."));
}

/* static */
void UINotificationMessage::warnAboutPublicKeyFileDoesntExist(const QString &strPath)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Public key missing ..."),
        QApplication::translate("UIMessageCenter", "Failed to open the public key file <nobr><b>%1</b></nobr>. "
                                                   "File doesn't exist.").arg(strPath));
}

/* static */
void UINotificationMessage::warnAboutPublicKeyFileIsOfTooLargeSize(const QString &strPath)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Public key too large ..."),
        QApplication::translate("UIMessageCenter", "Failed to open the public key file <nobr><b>%1</b></nobr>. File is too "
                                                   "large for the key.").arg(strPath));
}

/* static */
void UINotificationMessage::warnAboutPublicKeyFileIsntReadable(const QString &strPath)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Public key isn't readable ..."),
        QApplication::translate("UIMessageCenter", "Failed to open the public key file <nobr><b>%1</b></nobr>. Check file "
                                                   "permissions.").arg(strPath));
}

/* static */
void UINotificationMessage::warnAboutDHCPServerIsNotEnabled(const QString &strName)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "DHCP server isn't enabled ..."),
        QApplication::translate("UIMessageCenter", "Network <nobr><b>%1</b></nobr> is set to obtain the address "
                                                   "automatically but the corresponding DHCP server is not enabled.")
                                                   .arg(strName));
}

/* static */
void UINotificationMessage::warnAboutInvalidIPv4Address(const QString &strName)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Invalid IPv4 address ..."),
        QApplication::translate("UIMessageCenter", "Network <nobr><b>%1</b></nobr> does not "
                                                   "currently have a valid IPv4 address.")
                                                   .arg(strName));
}

/* static */
void UINotificationMessage::warnAboutInvalidIPv4Mask(const QString &strName)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Invalid IPv4 mask ..."),
        QApplication::translate("UIMessageCenter", "Network <nobr><b>%1</b></nobr> does not "
                                                   "currently have a valid IPv4 mask.")
                                                   .arg(strName));
}

/* static */
void UINotificationMessage::warnAboutInvalidIPv6Address(const QString &strName)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Invalid IPv6 address ..."),
        QApplication::translate("UIMessageCenter", "Network <nobr><b>%1</b></nobr> does not "
                                                   "currently have a valid IPv6 address.")
                                                   .arg(strName));
}

/* static */
void UINotificationMessage::warnAboutInvalidIPv6PrefixLength(const QString &strName)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Invalid IPv6 prefix length ..."),
        QApplication::translate("UIMessageCenter", "Network <nobr><b>%1</b></nobr> does not "
                                                   "currently have a valid IPv6 prefix length.")
                                                   .arg(strName));
}

/* static */
void UINotificationMessage::warnAboutInvalidDHCPServerAddress(const QString &strName)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Invalid DHCP server address ..."),
        QApplication::translate("UIMessageCenter", "Network <nobr><b>%1</b></nobr> does not "
                                                   "currently have a valid DHCP server address.")
                                                   .arg(strName));
}

/* static */
void UINotificationMessage::warnAboutInvalidDHCPServerMask(const QString &strName)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Invalid DHCP server mask ..."),
        QApplication::translate("UIMessageCenter", "Network <nobr><b>%1</b></nobr> does not "
                                                   "currently have a valid DHCP server mask.")
                                                   .arg(strName));
}

/* static */
void UINotificationMessage::warnAboutInvalidDHCPServerLowerAddress(const QString &strName)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Invalid DHCP lower address ..."),
        QApplication::translate("UIMessageCenter", "Network <nobr><b>%1</b></nobr> does not "
                                                   "currently have a valid DHCP server lower address bound.")
                                                   .arg(strName));
}

/* static */
void UINotificationMessage::warnAboutInvalidDHCPServerUpperAddress(const QString &strName)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Invalid DHCP upper address ..."),
        QApplication::translate("UIMessageCenter", "Network <nobr><b>%1</b></nobr> does not "
                                                   "currently have a valid DHCP server upper address bound.")
                                                   .arg(strName));
}

/* static */
void UINotificationMessage::warnAboutNoNameSpecified(const QString &strName)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "No name specified ..."),
        QApplication::translate("UIMessageCenter", "No new name specified for the network previously called <b>%1</b>.")
                                                   .arg(strName));
}

/* static */
void UINotificationMessage::warnAboutNameAlreadyBusy(const QString &strName)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Name already busy ..."),
        QApplication::translate("UIMessageCenter", "The name <b>%1</b> is being used for several networks.")
                                                   .arg(strName));
}

/* static */
void UINotificationMessage::warnAboutNoIPv4PrefixSpecified(const QString &strName)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "No IPv4 prefix specified ..."),
        QApplication::translate("UIMessageCenter", "No IPv4 prefix specified for the NAT network <b>%1</b>.")
                                                   .arg(strName));
}

/* static */
void UINotificationMessage::warnAboutNoIPv6PrefixSpecified(const QString &strName)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "No IPv6 prefix specified ..."),
        QApplication::translate("UIMessageCenter", "No IPv6 prefix specified for the NAT network <b>%1</b>.")
                                                   .arg(strName));
}

/* static */
bool UINotificationMessage::warnAboutIncorrectPort(QWidget *pParent)
{
    createMessage(
          QApplication::translate("UIMessageCenter", "Invalid port forwarding rules..."),
          QApplication::translate("UIMessageCenter", "None of the host or guest port values may be set to zero."),
          pParent);
    return false;
}

/* static */
bool UINotificationMessage::warnAboutIncorrectAddress(QWidget *pParent)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Invalid port forwarding rules..."),
        QApplication::translate("UIMessageCenter", "All of the host or guest address values should be correct or empty."),
        pParent);
    return false;
}

/* static */
bool UINotificationMessage::warnAboutEmptyGuestAddress(QWidget *pParent)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Invalid port forwarding rules..."),
        QApplication::translate("UIMessageCenter", "None of the guest address values may be empty."),
        pParent);
    return false;
}

/* static */
bool UINotificationMessage::warnAboutNameShouldBeUnique(QWidget *pParent)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Invalid port forwarding rules..."),
        QApplication::translate("UIMessageCenter", "Rule names should be unique."),
        pParent);
    return false;
}

/* static */
bool UINotificationMessage::warnAboutRulesConflict(QWidget *pParent)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Invalid port forwarding rules..."),
        QApplication::translate("UIMessageCenter", "Few rules have same host ports and conflicting IP addresses."),
        pParent);
    return false;
}

/* static */
void UINotificationMessage::warnAboutStateChange(QWidget *pParent)
{
    createMessage(QApplication::translate("UIMessageCenter", "State has changed ..."),
                  QApplication::translate("UIMessageCenter", "The virtual machine that you are changing has been started. "
                                                             "Only certain settings can be changed while a machine is running. "
                                                             "All other changes will be lost if you close this window now."),
                  "warnAboutStateChange",
                  QString(),
                  pParent);
}

/* static */
void UINotificationMessage::showRuntimeError(NotificationType emnNotificationType,
                                             const QString &strErrorId,
                                             const QString &strErrorMsg)
{
    /* Gather suitable severity and confirm id: */
    QString strSeverity;
    QByteArray autoConfimId = "showRuntimeError.";
    switch (emnNotificationType)
    {
        case NotificationType_Warning:
        {
            strSeverity = QApplication::translate("UIMessageCenter", "<nobr>Warning</nobr>", "runtime error info");
            autoConfimId += "warning.";
            break;
        }
        case NotificationType_Error:
        {
            strSeverity = QApplication::translate("UIMessageCenter", "<nobr>Non-Fatal Error</nobr>", "runtime error info");
            autoConfimId += "error.";
            break;
        }
        case NotificationType_Critical:
        {
            strSeverity = QApplication::translate("UIMessageCenter", "<nobr>Fatal Error</nobr>", "runtime error info");
            autoConfimId += "fatal.";
            break;
        }
        default:
            break;
    }
    autoConfimId += strErrorId.toUtf8();

    /* Format error-details: */
    QString formatted("<!--EOM-->");
    if (!strErrorMsg.isEmpty())
        formatted.prepend(QString("<p>%1.</p>").arg(UITranslator::emphasize(strErrorMsg)));
    if (!strErrorId.isEmpty())
        formatted += QString("<table bgcolor=%1 border=0 cellspacing=5 "
                             "cellpadding=0 width=100%>"
                             "<tr><td>%2</td><td>%3</td></tr>"
                             "<tr><td>%4</td><td>%5</td></tr>"
                             "</table>")
                             .arg(QApplication::palette().color(QPalette::Active, QPalette::Window).name(QColor::HexRgb))
                             .arg(QApplication::translate("UIMessageCenter", "<nobr>Error ID:</nobr>", "runtime error info"), strErrorId)
                             .arg(QApplication::translate("UIMessageCenter", "Severity:", "runtime error info"), strSeverity);
    if (!formatted.isEmpty())
        formatted = "<qt>" + formatted + "</qt>";

    /* Show the error: */
    switch (emnNotificationType)
    {
        case NotificationType_Warning:
        {
            createMessage(
                strSeverity,
                QApplication::translate("UIMessageCenter", "<p>The virtual machine execution ran into a non-fatal problem as "
                                                           "described below. We suggest that you take appropriate action to "
                                                           "prevent the problem from recurring.</p>") + formatted,
                autoConfimId.data());
            break;
        }
        case NotificationType_Error:
        {
            createMessage(
                strSeverity,
                QApplication::translate("UIMessageCenter", "<p>An error has occurred during virtual machine execution! The "
                                                           "error details are shown below. You may try to correct the error "
                                                           "and resume the virtual machine execution.</p>") + formatted,
                autoConfimId.data());
            break;
        }
        case NotificationType_Critical:
        {
            createBlockingMessage(
                strSeverity,
                QApplication::translate("UIMessageCenter", "<p>A fatal error has occurred during virtual machine execution! "
                                                           "The virtual machine will be powered off. Please copy the following "
                                                           "error message using the clipboard to help diagnose the "
                                                           "problem:</p>") + formatted,
                autoConfimId.data());
            break;
        }
        default:
            break;
    }
}

/* static */
void UINotificationMessage::cannotEnterSeamlessMode(quint64 uMinVRAM)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't enter seamless mode ..."),
        QApplication::translate("UIMessageCenter", "<p>Could not enter seamless mode due to insufficient guest video memory.</p>"
                                                   "<p>You should configure the virtual machine to have at least <b>%1</b> of "
                                                   "video memory.</p>")
                                                   .arg(UITranslator::formatSize(uMinVRAM)));
}

/* static */
void UINotificationMessage::cannotSwitchScreenInSeamless(quint64 uMinVRAM)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't switch to another screen ..."),
        QApplication::translate("UIMessageCenter", "<p>Could not change the guest screen to this host screen due to insufficient "
                                                   "guest video memory.</p><p>You should configure the virtual machine to have "
                                                   "at least <b>%1</b> of video memory.</p>")
                                                   .arg(UITranslator::formatSize(uMinVRAM)));
}

/* static */
void UINotificationMessage::cannotMountImage(const QString &strMachineName, const QString &strMediumName)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't mount image ..."),
        QApplication::translate("UIMessageCenter", "<p>Could not insert the <b>%1</b> disk image file into the virtual machine "
                                                   "<b>%2</b>, as the machine has no optical drives. Please add a drive using "
                                                   "the storage page of the virtual machine settings window.</p>")
                                                   .arg(strMediumName, strMachineName));
}

/* static */
void UINotificationMessage::cannotSendACPIToMachine()
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't send shutdown signal ..."),
        QApplication::translate("UIMessageCenter", "You are trying to shut down the guest with the ACPI power button. "
                                                   "This is currently not possible because the guest does not support "
                                                   "software shut down."));
}

/* static */
void UINotificationMessage::remindAboutAutoCapture()
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Auto capture keyboard ..."),
        QApplication::translate("UIMessageCenter", "<p>You have the <b>Auto capture keyboard</b> option turned on. "
                                                   "This will cause the Virtual Machine to automatically <b>capture</b> "
                                                   "the keyboard every time the VM window is activated and make it "
                                                   "unavailable to other applications running on your host machine: "
                                                   "when the keyboard is captured, all keystrokes (including system ones "
                                                   "like Alt-Tab) will be directed to the VM.</p>"
                                                   "<p>You can press the <b>host key combo</b> at any time to <b>uncapture</b> the "
                                                   "keyboard and mouse (if it is captured) and return them to normal "
                                                   "operation. The currently assigned host key combo is shown on the status bar "
                                                   "at the bottom of the Virtual Machine window. This icon, together "
                                                   "with the mouse icon placed nearby, indicate the current keyboard "
                                                   "and mouse capture state.</p>") +
        QApplication::translate("UIMessageCenter", "<p>The host key combo is currently defined as <b>%1</b>.</p>",
                                                   "additional message box paragraph")
                                                   .arg(UIHostCombo::toReadableString(gEDataManager->hostKeyCombination())),
        "remindAboutAutoCapture");
}

/* static */
void UINotificationMessage::remindAboutGuestAdditionsAreNotActive()
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Guest additions inactive ..."),
        QApplication::translate("UIMessageCenter", "<p>The VirtualBox Guest Additions do not appear to be available on this "
                                                   "virtual machine, and shared folders cannot be used without them. To use "
                                                   "shared folders inside the virtual machine, please install the Guest "
                                                   "Additions if they are not installed, or re-install them if they are not "
                                                   "working correctly, by selecting <b>Insert Guest Additions CD image</b> from "
                                                   "the <b>Devices</b> menu. If they are installed but the machine is not yet "
                                                   "fully started then shared folders will be available once it is.</p>"),
        "remindAboutGuestAdditionsAreNotActive");
}

/* static */
void UINotificationMessage::remindAboutMouseIntegration(bool fSupportsAbsolute)
{
    if (fSupportsAbsolute)
    {
        createMessage(
            QApplication::translate("UIMessageCenter", "Mouse integration ..."),
            QApplication::translate("UIMessageCenter", "<p>The Virtual Machine reports that the guest OS supports <b>mouse "
                                                       "pointer integration</b>. This means that you do not need to "
                                                       "<i>capture</i> the mouse pointer to be able to use it in your guest "
                                                       "OS -- all mouse actions you perform when the mouse pointer is over the "
                                                       "Virtual Machine's display are directly sent to the guest OS. If the "
                                                       "mouse is currently captured, it will be automatically uncaptured.</p>"
                                                       "<p>The mouse icon on the status bar will look "
                                                       "like&nbsp;<img src=:/mouse_seamless_16px.png/>&nbsp;to inform you that "
                                                       "mouse pointer integration is supported by the guest OS and is currently "
                                                       "turned on.</p><p><b>Note</b>: Some applications may behave incorrectly "
                                                       "in mouse pointer integration mode. You can always disable it for the "
                                                       "current session (and enable it again) by selecting the corresponding "
                                                       "action from the menu bar.</p>"),
            "remindAboutMouseIntegration");
    }
    else
    {
        createMessage(
            QApplication::translate("UIMessageCenter", "Mouse integration ..."),
            QApplication::translate("UIMessageCenter", "<p>The Virtual Machine reports that the guest OS does not support "
                                                       "<b>mouse pointer integration</b> in the current video mode. You need to "
                                                       "capture the mouse (by clicking over the VM display or pressing the host "
                                                       "key) in order to use the mouse inside the guest OS.</p>"),
            "remindAboutMouseIntegration");
    }
}

/* static */
void UINotificationMessage::remindAboutPausedVMInput()
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Paused VM input ..."),
        QApplication::translate("UIMessageCenter", "<p>The Virtual Machine is currently in the <b>Paused</b> state and not able "
                                                   "to see any keyboard or mouse input. If you want to continue to work inside "
                                                   "the VM, you need to resume it by selecting the corresponding action from the "
                                                   "menu bar.</p>"),
        "remindAboutPausedVMInput");
}

/* static */
void UINotificationMessage::forgetAboutPausedVMInput()
{
    destroyMessage("remindAboutPausedVMInput");
}

/* static */
void UINotificationMessage::remindAboutWrongColorDepth(ulong uRealBPP, ulong uWantedBPP)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Wrong color depth ..."),
        QApplication::translate("UIMessageCenter", "<p>The virtual screen is currently set to a <b>%1&nbsp;bit</b> color mode. "
                                                   "For better performance please change this to <b>%2&nbsp;bit</b>. This can "
                                                   "usually be done from the <b>Display</b> section of the guest operating "
                                                   "system's Control Panel or System Settings.</p>")
                                                   .arg(uRealBPP).arg(uWantedBPP),
        "remindAboutWrongColorDepth");
}

/* static */
void UINotificationMessage::forgetAboutWrongColorDepth()
{
    destroyMessage("remindAboutWrongColorDepth");
}

/* static */
void UINotificationMessage::warnAboutVBoxSVCUnavailable()
{
    createBlockingMessage(
        QApplication::translate("UIMessageCenter", "VBoxSVC not available!"),
        QApplication::translate("UIMessageCenter", "<p>A critical error has occurred while running the virtual "
                                                   "machine and the machine execution should be stopped.</p>"
                                                   "<p>For help, please see the Community section on "
                                                   "<a href=https://www.virtualbox.org>https://www.virtualbox.org</a> "
                                                   "or your support contract. Please provide the contents of the "
                                                   "log file <tt>VBox.log</tt>, "
                                                   "which you can find in the virtual machine log directory, "
                                                   "as well as a description of what you were doing when this error happened. "
                                                   "Note that you can also access the above file by selecting <b>Show Log</b> "
                                                   "from the <b>Machine</b> menu of the main VirtualBox window.</p>"
                                                   "<p>Close the message to power off the machine.</p>"));
}

/* static */
void UINotificationMessage::cannotAcquireVirtualBoxParameter(const CVirtualBox &comVBox,
                                                             QWidget *pParent /* = 0 */)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "VirtualBox failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to acquire VirtualBox parameter.") +
        UIErrorString::formatErrorInfo(comVBox),
        pParent);
}

/* static */
void UINotificationMessage::cannotAcquireApplianceParameter(const CAppliance &comAppliance,
                                                            QWidget *pParent)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Appliance failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to acquire appliance parameter.") +
        UIErrorString::formatErrorInfo(comAppliance),
        pParent);
}

/* static */
void UINotificationMessage::cannotAcquirePlatformParameter(const CPlatform &comPlatform)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Platform failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to acquire platform parameter.") +
        UIErrorString::formatErrorInfo(comPlatform));
}

/* static */
void UINotificationMessage::cannotAcquirePlatformPropertiesParameter(const CPlatformProperties &comProperties)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Platform properties failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to acquire platform properties parameter.") +
        UIErrorString::formatErrorInfo(comProperties));
}

/* static */
void UINotificationMessage::cannotAcquireSystemPropertiesParameter(const CSystemProperties &comProperties)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "System properties failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to acquire system properties parameter.") +
        UIErrorString::formatErrorInfo(comProperties));
}

/* static */
void UINotificationMessage::cannotAcquireExtensionPackManagerParameter(const CExtPackManager &comEPManager)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Extension Pack failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to acquire Extension Pack Manager parameter.") +
        UIErrorString::formatErrorInfo(comEPManager));
}

/* static */
void UINotificationMessage::cannotAcquireExtensionPackParameter(const CExtPack &comPackage)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Extension Pack failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to acquire Extension Pack parameter.") +
        UIErrorString::formatErrorInfo(comPackage));
}

/* static */
void UINotificationMessage::cannotAcquireHostParameter(const CHost &comHost)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Host failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to acquire host parameter.") +
        UIErrorString::formatErrorInfo(comHost));
}

/* static */
void UINotificationMessage::cannotAcquireStorageControllerParameter(const CStorageController &comStorageController)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Storage controller failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to acquire storage controller parameter.") +
        UIErrorString::formatErrorInfo(comStorageController));
}

/* static */
void UINotificationMessage::cannotChangeStorageControllerParameter(const CStorageController &comStorageController)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Storage controller failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to change storage controller parameter.") +
        UIErrorString::formatErrorInfo(comStorageController));
}

/* static */
void UINotificationMessage::cannotAcquireMediumAttachmentParameter(const CMediumAttachment &comMediumAttachment)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Medium attachment failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to acquire medium attachment parameter.") +
        UIErrorString::formatErrorInfo(comMediumAttachment));
}

/* static */
void UINotificationMessage::cannotAcquireMediumParameter(const CMedium &comMedium)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Medium failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to acquire medium parameter.") +
        UIErrorString::formatErrorInfo(comMedium));
}

/* static */
void UINotificationMessage::cannotAcquireSessionParameter(const CSession &comSession)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Session failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to acquire session parameter.") +
        UIErrorString::formatErrorInfo(comSession));
}

/* static */
void UINotificationMessage::cannotAcquireMachineParameter(const CMachine &comMachine)
{
    /* Do not show error for the E_NOTIMPL case, just add it to the log: */
    if (comMachine.lastRC() == E_NOTIMPL)
    {
        LogRel(("GUI: IMachine getter lastRC == E_NOTIMPL, skipping ...\n"));
        return;
    }

    createMessage(
        QApplication::translate("UIMessageCenter", "Machine failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to acquire machine parameter.") +
        UIErrorString::formatErrorInfo(comMachine));
}

/* static */
void UINotificationMessage::cannotAcquireMachineDebuggerParameter(const CMachineDebugger &comMachineDebugger)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Debugger failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to acquire machine debugger parameter.") +
        UIErrorString::formatErrorInfo(comMachineDebugger));
}

/* static */
void UINotificationMessage::cannotAcquireGraphicsAdapterParameter(const CGraphicsAdapter &comAdapter)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Graphics adapter failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to acquire graphics adapter parameter.") +
        UIErrorString::formatErrorInfo(comAdapter));
}

/* static */
void UINotificationMessage::cannotAcquireAudioSettingsParameter(const CAudioSettings &comSettings)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Audio settings failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to acquire audio settings parameter.") +
        UIErrorString::formatErrorInfo(comSettings));
}

/* static */
void UINotificationMessage::cannotAcquireAudioAdapterParameter(const CAudioAdapter &comAdapter)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Audio adapter failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to acquire audio adapter parameter.") +
        UIErrorString::formatErrorInfo(comAdapter));
}

/* static */
void UINotificationMessage::cannotAcquireNetworkAdapterParameter(const CNetworkAdapter &comAdapter)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Network adapter failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to acquire network adapter parameter.") +
        UIErrorString::formatErrorInfo(comAdapter));
}

/* static */
void UINotificationMessage::cannotAcquireConsoleParameter(const CConsole &comConsole)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Console failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to acquire console parameter.") +
        UIErrorString::formatErrorInfo(comConsole));
}

/* static */
void UINotificationMessage::cannotAcquireGuestParameter(const CGuest &comGuest)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Guest failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to acquire guest parameter.") +
        UIErrorString::formatErrorInfo(comGuest));
}

/* static */
void UINotificationMessage::cannotAcquireGuestOSTypeParameter(const CGuestOSType &comGuestOSType)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Guest OS type failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to acquire guest OS type parameter.") +
        UIErrorString::formatErrorInfo(comGuestOSType));
}

/* static */
void UINotificationMessage::cannotAcquireSnapshotParameter(const CSnapshot &comSnapshot)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Snapshot failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to acquire snapshot parameter.") +
        UIErrorString::formatErrorInfo(comSnapshot));
}

/* static */
void UINotificationMessage::cannotAcquireDHCPServerParameter(const CDHCPServer &comServer)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "DHCP server failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to acquire DHCP server parameter.") +
        UIErrorString::formatErrorInfo(comServer));
}

/* static */
void UINotificationMessage::cannotAcquireCloudNetworkParameter(const CCloudNetwork &comNetwork)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Cloud failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to acquire cloud network parameter.") +
        UIErrorString::formatErrorInfo(comNetwork));
}

/* static */
void UINotificationMessage::cannotAcquireHostNetworkInterfaceParameter(const CHostNetworkInterface &comInterface)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Host network interface failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to acquire host network interface parameter.") +
        UIErrorString::formatErrorInfo(comInterface));
}

/* static */
void UINotificationMessage::cannotAcquireHostOnlyNetworkParameter(const CHostOnlyNetwork &comNetwork)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Host only network failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to acquire host only network parameter.") +
        UIErrorString::formatErrorInfo(comNetwork));
}

/* static */
void UINotificationMessage::cannotAcquireNATNetworkParameter(const CNATNetwork &comNetwork)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "NAT network failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to acquire NAT network parameter.") +
        UIErrorString::formatErrorInfo(comNetwork));
}

/* static */
void UINotificationMessage::cannotAcquireDisplayParameter(const CDisplay &comDisplay)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Display failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to acquire display parameter.") +
        UIErrorString::formatErrorInfo(comDisplay));
}

/* static */
void UINotificationMessage::cannotAcquireUpdateAgentParameter(const CUpdateAgent &comAgent)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Update failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to acquire update agent parameter.") +
        UIErrorString::formatErrorInfo(comAgent));
}

/* static */
void UINotificationMessage::cannotAcquireMouseParameter(const CMouse &comMouse)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Mouse failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to acquire mouse parameter.") +
        UIErrorString::formatErrorInfo(comMouse));
}

/* static */
void UINotificationMessage::cannotAcquireEmulatedUSBParameter(const CEmulatedUSB &comDispatcher)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Emulated USB failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to acquire emulated USB parameter.") +
        UIErrorString::formatErrorInfo(comDispatcher));
}

/* static */
void UINotificationMessage::cannotAcquireRecordingSettingsParameter(const CRecordingSettings &comSettings)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Recording settings failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to acquire recording settings parameter.") +
        UIErrorString::formatErrorInfo(comSettings));
}

/* static */
void UINotificationMessage::cannotAcquireVRDEServerParameter(const CVRDEServer &comServer)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "VRDE server failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to acquire VRDE server parameter.") +
        UIErrorString::formatErrorInfo(comServer));
}

/* static */
void UINotificationMessage::cannotAcquireVRDEServerInfoParameter(const CVRDEServerInfo &comServerInfo)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "VRDE server info failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to acquire VRDE server info parameter.") +
        UIErrorString::formatErrorInfo(comServerInfo));
}

/* static */
void UINotificationMessage::cannotAcquireVirtualSystemDescriptionParameter(const CVirtualSystemDescription &comVsd,
                                                                           QWidget *pParent)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "VSD failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to acquire VSD parameter.") +
        UIErrorString::formatErrorInfo(comVsd),
        pParent);
}

/* static */
void UINotificationMessage::cannotAcquireVirtualSystemDescriptionFormParameter(const CVirtualSystemDescriptionForm &comVsdForm,
                                                                               QWidget *pParent)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "VSD form failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to acquire VSD form parameter.") +
        UIErrorString::formatErrorInfo(comVsdForm),
        pParent);
}

/* static */
void UINotificationMessage::cannotAcquireCloudProviderManagerParameter(const CCloudProviderManager &comCloudProviderManager,
                                                                       QWidget *pParent)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Cloud failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to acquire cloud provider manager parameter.") +
        UIErrorString::formatErrorInfo(comCloudProviderManager),
        pParent);
}

/* static */
void UINotificationMessage::cannotAcquireCloudProviderParameter(const CCloudProvider &comCloudProvider,
                                                                QWidget *pParent)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Cloud failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to acquire cloud provider parameter.") +
        UIErrorString::formatErrorInfo(comCloudProvider),
        pParent);
}

/* static */
void UINotificationMessage::cannotAcquireCloudProfileParameter(const CCloudProfile &comCloudProfile,
                                                               QWidget *pParent)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Cloud failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to acquire cloud profile parameter.") +
        UIErrorString::formatErrorInfo(comCloudProfile),
        pParent);
}

/* static */
void UINotificationMessage::cannotAcquireCloudMachineParameter(const CCloudMachine &comCloudMachine,
                                                               QWidget *pParent)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Cloud failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to acquire cloud machine parameter.") +
        UIErrorString::formatErrorInfo(comCloudMachine),
        pParent);
}

/* static */
void UINotificationMessage::cannotChangeHostParameter(const CHost &comHost, QWidget *pParent /* = 0 */)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Host failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to change host parameter.") +
        UIErrorString::formatErrorInfo(comHost),
        pParent);
}

/* static */
void UINotificationMessage::cannotChangeSystemProperties(const CSystemProperties &comProperties, QWidget *pParent /* = 0 */)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "System properties failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to change system properties.") +
        UIErrorString::formatErrorInfo(comProperties),
        pParent);
}

/* static */
void UINotificationMessage::cannotChangeMediumParameter(const CMedium &comMedium)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Medium failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to change the parameter of the medium <b>%1</b>.")
                                                   .arg(CMedium(comMedium).GetLocation()) +
        UIErrorString::formatErrorInfo(comMedium));
}

/* static */
void UINotificationMessage::cannotChangeMachineParameter(const CMachine &comMachine)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Machine failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to change the parameter of the virtual machine <b>%1</b>.")
                                                   .arg(CMachine(comMachine).GetName()) +
        UIErrorString::formatErrorInfo(comMachine));
}

/* static */
void UINotificationMessage::cannotChangeMachineDebuggerParameter(const CMachineDebugger &comMachineDebugger)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Debugger failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to change machine debugger parameter.") +
        UIErrorString::formatErrorInfo(comMachineDebugger));
}

/* static */
void UINotificationMessage::cannotChangeGraphicsAdapterParameter(const CGraphicsAdapter &comAdapter)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Graphics adapter failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to change graphics adapter parameter.") +
        UIErrorString::formatErrorInfo(comAdapter));
}

/* static */
void UINotificationMessage::cannotChangeAudioAdapterParameter(const CAudioAdapter &comAdapter)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Audio adapter failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to change audio adapter parameter.") +
        UIErrorString::formatErrorInfo(comAdapter));
}

/* static */
void UINotificationMessage::cannotChangeNetworkAdapterParameter(const CNetworkAdapter &comAdapter)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Network adapter failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to change network adapter parameter.") +
        UIErrorString::formatErrorInfo(comAdapter));
}

/* static */
void UINotificationMessage::cannotChangeDHCPServerParameter(const CDHCPServer &comServer)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "DHCP server failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to change DHCP server parameter.") +
        UIErrorString::formatErrorInfo(comServer));
}

/* static */
void UINotificationMessage::cannotChangeCloudNetworkParameter(const CCloudNetwork &comNetwork)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Cloud failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to change cloud network parameter.") +
        UIErrorString::formatErrorInfo(comNetwork));
}

/* static */
void UINotificationMessage::cannotChangeHostNetworkInterfaceParameter(const CHostNetworkInterface &comInterface)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Host network interface failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to change host network interface parameter.") +
        UIErrorString::formatErrorInfo(comInterface));
}

/* static */
void UINotificationMessage::cannotChangeHostOnlyNetworkParameter(const CHostOnlyNetwork &comNetwork)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Host only network failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to change host only network parameter.") +
        UIErrorString::formatErrorInfo(comNetwork));
}

/* static */
void UINotificationMessage::cannotChangeNATNetworkParameter(const CNATNetwork &comNetwork)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "NAT network failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to change NAT network parameter.") +
        UIErrorString::formatErrorInfo(comNetwork));
}

/* static */
void UINotificationMessage::cannotChangeDisplayParameter(const CDisplay &comDisplay)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Display failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to change display parameter.") +
        UIErrorString::formatErrorInfo(comDisplay));
}

/* static */
void UINotificationMessage::cannotChangeCloudProfileParameter(const CCloudProfile &comProfile)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Cloud failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to change cloud profile parameter.") +
        UIErrorString::formatErrorInfo(comProfile));
}

/* static */
void UINotificationMessage::cannotChangeUpdateAgentParameter(const CUpdateAgent &comAgent)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Update failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to change update agent parameter.") +
        UIErrorString::formatErrorInfo(comAgent));
}

/* static */
void UINotificationMessage::cannotChangeKeyboardParameter(const CKeyboard &comKeyboard)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Keyboard failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to change keyboard parameter.") +
        UIErrorString::formatErrorInfo(comKeyboard));
}

/* static */
void UINotificationMessage::cannotChangeVirtualSystemDescriptionParameter(const CVirtualSystemDescription &comVsd,
                                                                          QWidget *pParent)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "VSD failure ..."),
        QApplication::translate("UIMessageCenter", "Failed to change VSD parameter.") +
        UIErrorString::formatErrorInfo(comVsd),
        pParent);
}

/* static */
void UINotificationMessage::cannotEnumerateHostUSBDevices(const CHost &comHost)
{
    /* Refer users to manual's trouble shooting section depending on the host platform: */
    QString strHelpKeyword;
#if defined(RT_OS_LINUX)
    strHelpKeyword = "ts_usb-linux";
#elif defined(RT_OS_WINDOWS)
    strHelpKeyword = "ts_win-guests";
#elif defined(RT_OS_SOLARIS)
    strHelpKeyword = "ts_sol-guests";
#elif defined(RT_OS_DARWIN)
#endif

    createMessage(
        QApplication::translate("UIMessageCenter", "Can't enumerate USB devices ..."),
        QApplication::translate("UIMessageCenter", "Failed to enumerate host USB devices.") +
        UIErrorString::formatErrorInfo(comHost),
        "cannotEnumerateHostUSBDevices",
        strHelpKeyword);
}

/* static */
void UINotificationMessage::cannotAccessUSBSubsystem(const CMachine &comMachine, QWidget *pParent)
{
    /* If IMachine::GetUSBController() return E_NOTIMPL, it means the USB support is intentionally
     * missing (as in the OSE version). Don't show the error message in this case. */
    COMResult res(comMachine);
    if (res.rc() == E_NOTIMPL)
        return;

    createMessage(QApplication::translate("UIMessageCenter", "Can't access USB subsystem ..."),
                  QApplication::translate("UIMessageCenter", "Failed to access the USB subsystem.") +
                  UIErrorString::formatErrorInfo(res),
                  "cannotAccessUSBSubsystem",
                  QString(),
                  pParent);
}

/* static */
void UINotificationMessage::cannotOpenMedium(const CVirtualBox &comVBox,
                                             const QString &strLocation,
                                             QWidget *pParent /* = 0 */)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't open medium ..."),
        QApplication::translate("UIMessageCenter", "Failed to open the disk image file <nobr><b>%1</b></nobr>.")
                                                   .arg(strLocation) +
        UIErrorString::formatErrorInfo(comVBox),
        pParent);
}

/* static */
void UINotificationMessage::cannotPauseMachine(const CConsole &comConsole)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't pause machine ..."),
        QApplication::translate("UIMessageCenter", "Failed to pause the execution of the virtual machine <b>%1</b>.")
                                                   .arg(CConsole(comConsole).GetMachine().GetName()) +
        UIErrorString::formatErrorInfo(comConsole));
}

/* static */
void UINotificationMessage::cannotResumeMachine(const CConsole &comConsole)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't resume machine ..."),
        QApplication::translate("UIMessageCenter", "Failed to resume the execution of the virtual machine <b>%1</b>.")
                                                   .arg(CConsole(comConsole).GetMachine().GetName()) +
        UIErrorString::formatErrorInfo(comConsole));
}

/* static */
void UINotificationMessage::cannotACPIShutdownMachine(const CConsole &comConsole)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't shut down machine ..."),
        QApplication::translate("UIMessageCenter", "Failed to send the ACPI power button press event to the virtual machine "
                                                   "<b>%1</b>.").arg(CConsole(comConsole).GetMachine().GetName()) +
        UIErrorString::formatErrorInfo(comConsole));
}

/* static */
void UINotificationMessage::cannotResetMachine(const CConsole &comConsole)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't reset machine ..."),
        QApplication::translate("UIMessageCenter", "Failed to reset the virtual machine "
                                                   "<b>%1</b>.").arg(CConsole(comConsole).GetMachine().GetName()) +
        UIErrorString::formatErrorInfo(comConsole));
}

/* static */
void UINotificationMessage::cannotSetGroups(const CMachine &comMachine)
{
    QString strName = CMachine(comMachine).GetName();
    if (strName.isEmpty())
        strName = QFileInfo(CMachine(comMachine).GetSettingsFilePath()).baseName();

    createMessage(
        QApplication::translate("UIMessageCenter", "Failed to set groups ..."),
        QApplication::translate("UIMessageCenter", "Failed to set groups of the virtual "
                                                   "machine <b>%1</b>.").arg(strName) +
        UIErrorString::formatErrorInfo(comMachine),
        0);
}

/* static */
void UINotificationMessage::cannotCreateAppliance(const CVirtualBox &comVBox,
                                                  QWidget *pParent)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't create appliance ..."),
        QApplication::translate("UIMessageCenter", "Failed to create appliance.") +
        UIErrorString::formatErrorInfo(comVBox),
        pParent);
}

/* static */
void UINotificationMessage::cannotRegisterMachine(const CVirtualBox &comVBox,
                                                  const QString &strName,
                                                  QWidget *pParent /* = 0 */)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't register machine ..."),
        QApplication::translate("UIMessageCenter", "Failed to register machine <b>%1</b>.")
                                                   .arg(strName) +
        UIErrorString::formatErrorInfo(comVBox),
        pParent);
}

/* static */
void UINotificationMessage::cannotCreateMachine(const CVirtualBox &comVBox,
                                                QWidget *pParent)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't create machine ..."),
        QApplication::translate("UIMessageCenter", "Failed to create machine.") +
        UIErrorString::formatErrorInfo(comVBox),
        pParent);
}

/* static */
void UINotificationMessage::cannotFindMachineById(const CVirtualBox &comVBox,
                                                  const QUuid &uMachineId,
                                                  QWidget *pParent)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't find machine ..."),
        QApplication::translate("UIMessageCenter", "Failed to find the machine with following ID: <nobr><b>%1</b></nobr>.")
                                                   .arg(uMachineId.toString()) +
        UIErrorString::formatErrorInfo(comVBox),
        pParent);
}

/* static */
void UINotificationMessage::cannotOpenMachine(const CVirtualBox &comVBox, const QString &strLocation)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't open machine ..."),
        QApplication::translate("UIMessageCenter", "Failed to open virtual machine located in %1.")
                                                   .arg(strLocation) +
        UIErrorString::formatErrorInfo(comVBox));
}

/* static */
void UINotificationMessage::cannotCreateMediumStorage(const CVirtualBox &comVBox,
                                                      const QString &strPath,
                                                      QWidget *pParent)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't create medium storage ..."),
        QApplication::translate("UIMessageCenter", "Failed to create medium storage at <nobr><b>%1</b></nobr>.")
                                                   .arg(strPath) +
        UIErrorString::formatErrorInfo(comVBox),
        pParent);
}

/* static */
void UINotificationMessage::cannotGetExtensionPackManager(const CVirtualBox &comVBox)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't get Extension Pack Manager ..."),
        QApplication::translate("UIMessageCenter", "Failed to acquire Extension Pack Manager.") +
        UIErrorString::formatErrorInfo(comVBox));
}

/* static */
void UINotificationMessage::cannotCreateVfsExplorer(const CAppliance &comAppliance, QWidget *pParent)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't create VFS explorer ..."),
        QApplication::translate("UIMessageCenter", "Failed to create VFS explorer to check files.") +
        UIErrorString::formatErrorInfo(comAppliance),
        pParent);
}

/* static */
void UINotificationMessage::cannotAddDiskEncryptionPassword(const CAppliance &comAppliance, QWidget *pParent)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Bad password ..."),
        QApplication::translate("UIMessageCenter", "Bad password or authentication failure.") +
        UIErrorString::formatErrorInfo(comAppliance),
        pParent);
}

/* static */
void UINotificationMessage::cannotInterpretAppliance(const CAppliance &comAppliance, QWidget *pParent)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't interpret appliance ..."),
        QApplication::translate("UIMessageCenter", "Failed to interpret appliance being imported.") +
        UIErrorString::formatErrorInfo(comAppliance),
        pParent);
}

/* static */
void UINotificationMessage::cannotCreateVirtualSystemDescription(const CAppliance &comAppliance, QWidget *pParent)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't create VSD ..."),
        QApplication::translate("UIMessageCenter", "Failed to create VSD.") +
        UIErrorString::formatErrorInfo(comAppliance),
        pParent);
}

/* static */
void UINotificationMessage::cannotOpenExtPack(const CExtPackManager &comExtPackManager, const QString &strFilename)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't open extension pack ..."),
        QApplication::translate("UIMessageCenter", "Failed to open the Extension Pack <b>%1</b>.")
                                                   .arg(strFilename) +
        UIErrorString::formatErrorInfo(comExtPackManager));
}

/* static */
void UINotificationMessage::cannotReadExtPack(const CExtPackFile &comExtPackFile, const QString &strFilename)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't read extension pack ..."),
        QApplication::translate("UIMessageCenter", "Failed to read the Extension Pack <b>%1</b>.")
                                                   .arg(strFilename) +
        comExtPackFile.GetWhyUnusable());
}

/* static */
void UINotificationMessage::cannotFindCloudNetwork(const CVirtualBox &comVBox, const QString &strNetworkName)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't find cloud network ..."),
        QApplication::translate("UIMessageCenter", "Unable to find the cloud network <b>%1</b>.")
                                                   .arg(strNetworkName) +
        UIErrorString::formatErrorInfo(comVBox));
}

/* static */
void UINotificationMessage::cannotFindHostNetworkInterface(const CHost &comHost, const QString &strInterfaceName)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't find host network interface ..."),
        QApplication::translate("UIMessageCenter", "Unable to find the host network interface <b>%1</b>.")
                                                   .arg(strInterfaceName) +
        UIErrorString::formatErrorInfo(comHost));
}

/* static */
void UINotificationMessage::cannotFindHostOnlyNetwork(const CVirtualBox &comVBox, const QString &strNetworkName)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't find host only network ..."),
        QApplication::translate("UIMessageCenter", "Unable to find the host only network <b>%1</b>.")
                                                   .arg(strNetworkName) +
        UIErrorString::formatErrorInfo(comVBox));
}

/* static */
void UINotificationMessage::cannotFindNATNetwork(const CVirtualBox &comVBox, const QString &strNetworkName)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't find NAT network ..."),
        QApplication::translate("UIMessageCenter", "Unable to find the NAT network <b>%1</b>.")
                                                   .arg(strNetworkName) +
        UIErrorString::formatErrorInfo(comVBox));
}

/* static */
void UINotificationMessage::cannotCreateDHCPServer(const CVirtualBox &comVBox, const QString &strInterfaceName)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't create DHCP server ..."),
        QApplication::translate("UIMessageCenter", "Failed to create a DHCP server for the network interface <b>%1</b>.")
                                                   .arg(strInterfaceName) +
        UIErrorString::formatErrorInfo(comVBox));
}

/* static */
void UINotificationMessage::cannotRemoveDHCPServer(const CVirtualBox &comVBox, const QString &strInterfaceName)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't remove DHCP server ..."),
        QApplication::translate("UIMessageCenter", "Failed to remove the DHCP server for the network interface <b>%1</b>.")
                                                   .arg(strInterfaceName) +
        UIErrorString::formatErrorInfo(comVBox));
}

/* static */
void UINotificationMessage::cannotCreateCloudNetwork(const CVirtualBox &comVBox)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't create cloud network ..."),
        QApplication::translate("UIMessageCenter", "Failed to create a cloud network.") +
        UIErrorString::formatErrorInfo(comVBox));
}

/* static */
void UINotificationMessage::cannotRemoveCloudNetwork(const CVirtualBox &comVBox, const QString &strNetworkName)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't remove cloud network ..."),
        QApplication::translate("UIMessageCenter", "Failed to remove the cloud network <b>%1</b>.")
                                                   .arg(strNetworkName) +
        UIErrorString::formatErrorInfo(comVBox));
}

/* static */
void UINotificationMessage::cannotCreateHostOnlyNetwork(const CVirtualBox &comVBox)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't create host only network ..."),
        QApplication::translate("UIMessageCenter", "Failed to create a host only network.") +
        UIErrorString::formatErrorInfo(comVBox));
}

/* static */
void UINotificationMessage::cannotRemoveHostOnlyNetwork(const CVirtualBox &comVBox, const QString &strNetworkName)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't remove host only network ..."),
        QApplication::translate("UIMessageCenter", "Failed to remove the host only network <b>%1</b>.")
                                                   .arg(strNetworkName) +
        UIErrorString::formatErrorInfo(comVBox));
}

/* static */
void UINotificationMessage::cannotCreateNATNetwork(const CVirtualBox &comVBox)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't create NAT network ..."),
        QApplication::translate("UIMessageCenter", "Failed to create a NAT network.") +
        UIErrorString::formatErrorInfo(comVBox));
}

/* static */
void UINotificationMessage::cannotRemoveNATNetwork(const CVirtualBox &comVBox, const QString &strNetworkName)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't remove NAT network ..."),
        QApplication::translate("UIMessageCenter", "Failed to remove the NAT network <b>%1</b>.")
                                                   .arg(strNetworkName) +
        UIErrorString::formatErrorInfo(comVBox));
}

/* static */
void UINotificationMessage::cannotCreateCloudProfile(const CCloudProvider &comProvider)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't create cloud profile ..."),
        QApplication::translate("UIMessageCenter", "Failed to create cloud profile.") +
        UIErrorString::formatErrorInfo(comProvider));
}

/* static */
void UINotificationMessage::cannotRemoveCloudProfile(const CCloudProfile &comProfile)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't remove cloud profile ..."),
        QApplication::translate("UIMessageCenter", "Failed to remove cloud profile.") +
        UIErrorString::formatErrorInfo(comProfile));
}

/* static */
void UINotificationMessage::cannotSaveCloudProfiles(const CCloudProvider &comProvider)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't save cloud profiles ..."),
        QApplication::translate("UIMessageCenter", "Failed to save cloud profiles.") +
        UIErrorString::formatErrorInfo(comProvider));
}

/* static */
void UINotificationMessage::cannotImportCloudProfiles(const CCloudProvider &comProvider)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't import cloud profiles ..."),
        QApplication::translate("UIMessageCenter", "Failed to import cloud profiles.") +
        UIErrorString::formatErrorInfo(comProvider));
}

/* static */
void UINotificationMessage::cannotRefreshCloudMachine(const CCloudMachine &comMachine)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't refresh cloud machine ..."),
        QApplication::translate("UIMessageCenter", "Failed to refresh cloud machine.") +
        UIErrorString::formatErrorInfo(comMachine));
}

/* static */
void UINotificationMessage::cannotRefreshCloudMachine(const CProgress &comProgress)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't refresh cloud machine ..."),
        QApplication::translate("UIMessageCenter", "Failed to refresh cloud machine.") +
        UIErrorString::formatErrorInfo(comProgress));
}

/* static */
void UINotificationMessage::cannotCreateCloudClient(const CCloudProfile &comProfile, QWidget *pParent)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't create cloud client ..."),
        QApplication::translate("UIMessageCenter", "Failed to create cloud client.") +
        UIErrorString::formatErrorInfo(comProfile),
        pParent);
}

/* static */
void UINotificationMessage::cannotCloseMedium(const CMedium &comMedium)
{
    /* Show the error: */
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't close medium ..."),
        QApplication::translate("UIMessageCenter", "Failed to close the disk image file <nobr><b>%1</b></nobr>.")
                                                   .arg(CMedium(comMedium).GetLocation()) +
        UIErrorString::formatErrorInfo(comMedium));
}

/* static */
void UINotificationMessage::cannotDiscardSavedState(const CMachine &comMachine)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't discard saved state ..."),
        QApplication::translate("UIMessageCenter", "Failed to discard the saved state of the virtual machine <b>%1</b>.")
                                                   .arg(CMachine(comMachine).GetName()) +
        UIErrorString::formatErrorInfo(comMachine));
}

/* static */
void UINotificationMessage::cannotRemoveMachine(const CMachine &comMachine, QWidget *pParent /* = 0 */)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't remove machine ..."),
        QApplication::translate("UIMessageCenter", "Failed to remove the virtual machine <b>%1</b>.")
                                                   .arg(CMachine(comMachine).GetName()) +
        UIErrorString::formatErrorInfo(comMachine),
        pParent);
}

/* static */
void UINotificationMessage::cannotExportMachine(const CMachine &comMachine, QWidget *pParent)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't export machine ..."),
        QApplication::translate("UIMessageCenter", "Failed to export virtual machine <b>%1</b>.")
                                                   .arg(CMachine(comMachine).GetName()) +
        UIErrorString::formatErrorInfo(comMachine),
        pParent);
}

/* static */
void UINotificationMessage::cannotAttachDevice(const CMachine &comMachine,
                                               UIMediumDeviceType enmType,
                                               const QString &strLocation,
                                               const StorageSlot &storageSlot,
                                               QWidget *pParent /* = 0 */)
{
    QString strMessage;
    switch (enmType)
    {
        case UIMediumDeviceType_HardDisk:
        {
            strMessage = QApplication::translate("UIMessageCenter", "Failed to attach the hard disk (<nobr><b>%1</b></nobr>) to "
                                                                    "the slot <i>%2</i> of the machine <b>%3</b>.")
                                                                    .arg(strLocation)
                                                                    .arg(gpConverter->toString(storageSlot))
                                                                    .arg(CMachine(comMachine).GetName());
            break;
        }
        case UIMediumDeviceType_DVD:
        {
            strMessage = QApplication::translate("UIMessageCenter", "Failed to attach the optical drive (<nobr><b>%1</b></nobr>) "
                                                                    "to the slot <i>%2</i> of the machine <b>%3</b>.")
                                                                    .arg(strLocation)
                                                                    .arg(gpConverter->toString(storageSlot))
                                                                    .arg(CMachine(comMachine).GetName());
            break;
        }
        case UIMediumDeviceType_Floppy:
        {
            strMessage = QApplication::translate("UIMessageCenter", "Failed to attach the floppy drive (<nobr><b>%1</b></nobr>) "
                                                                    "to the slot <i>%2</i> of the machine <b>%3</b>.")
                                                                    .arg(strLocation)
                                                                    .arg(gpConverter->toString(storageSlot))
                                                                    .arg(CMachine(comMachine).GetName());
            break;
        }
        default:
            break;
    }
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't attach device ..."),
        strMessage + UIErrorString::formatErrorInfo(comMachine),
        pParent);
}

/* static */
void UINotificationMessage::cannotDetachDevice(const CMachine &comMachine,
                                               UIMediumDeviceType enmType,
                                               const QString &strLocation,
                                               const StorageSlot &storageSlot,
                                               QWidget *pParent /* = 0 */)
{
    QString strMessage;
    switch (enmType)
    {
        case UIMediumDeviceType_HardDisk:
        {
            strMessage = QApplication::translate("UIMessageCenter", "Failed to detach the hard disk (<nobr><b>%1</b></nobr>) "
                                                                    "from the slot <i>%2</i> of the machine <b>%3</b>.")
                                                                    .arg(strLocation)
                                                                    .arg(gpConverter->toString(storageSlot))
                                                                    .arg(CMachine(comMachine).GetName());
            break;
        }
        case UIMediumDeviceType_DVD:
        {
            strMessage = QApplication::translate("UIMessageCenter", "Failed to detach the optical drive (<nobr><b>%1</b></nobr>) "
                                                                    "from the slot <i>%2</i> of the machine <b>%3</b>.")
                                                                    .arg(strLocation)
                                                                    .arg(gpConverter->toString(storageSlot))
                                                                    .arg(CMachine(comMachine).GetName());
            break;
        }
        case UIMediumDeviceType_Floppy:
        {
            strMessage = QApplication::translate("UIMessageCenter", "Failed to detach the floppy drive (<nobr><b>%1</b></nobr>) "
                                                                    "from the slot <i>%2</i> of the machine <b>%3</b>.")
                                                                    .arg(strLocation)
                                                                    .arg(gpConverter->toString(storageSlot))
                                                                    .arg(CMachine(comMachine).GetName());
            break;
        }
        default:
            break;
    }
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't detach device ..."),
        strMessage + UIErrorString::formatErrorInfo(comMachine),
        pParent);
}


/* static */
void UINotificationMessage::cannotFindSnapshotById(const CMachine &comMachine, const QUuid &uId)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't find snapshot ..."),
        QApplication::translate("UIMessageCenter", "Failed to find snapshot with ID=<b>%1</b>.")
                                                   .arg(uId.toString()) +
        UIErrorString::formatErrorInfo(comMachine));
}

/* static */
void UINotificationMessage::cannotFindSnapshotByName(const CMachine &comMachine,
                                                     const QString &strName,
                                                     QWidget *pParent)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't find snapshot ..."),
        QApplication::translate("UIMessageCenter", "Failed to find snapshot with name=<b>%1</b>.")
                                                   .arg(strName) +
        UIErrorString::formatErrorInfo(comMachine),
        pParent);
}

/* static */
void UINotificationMessage::cannotChangeSnapshot(const CSnapshot &comSnapshot,
                                                 const QString &strSnapshotName,
                                                 const QString &strMachineName)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't change snapshot ..."),
        QApplication::translate("UIMessageCenter", "Failed to change the snapshot <b>%1</b> of the virtual machine <b>%2</b>.")
                                                   .arg(strSnapshotName, strMachineName) +
        UIErrorString::formatErrorInfo(comSnapshot));
}

/* static */
void UINotificationMessage::cannotRunUnattendedGuestInstall(const CUnattended &comUnattended)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't run guest install ..."),
        QApplication::translate("UIMessageCenter", "Failed to run unattended guest installation.") +
        UIErrorString::formatErrorInfo(comUnattended));
}

/* static */
void UINotificationMessage::cannotStartMachine(const CConsole &comConsole, const QString &strName)
{
    createBlockingMessage(
        QApplication::translate("UIMessageCenter", "Can't run virtual machine ..."),
        QApplication::translate("UIMessageCenter", "Failed to start the virtual machine <b>%1</b>.").arg(strName) +
        UIErrorString::formatErrorInfo(comConsole));
}

/* static */
void UINotificationMessage::cannotStartMachine(const CProgress &comProgress, const QString &strName)
{
    createBlockingMessage(
        QApplication::translate("UIMessageCenter", "Can't run virtual machine ..."),
        QApplication::translate("UIMessageCenter", "Failed to start the virtual machine <b>%1</b>.").arg(strName) +
        UIErrorString::formatErrorInfo(comProgress));
}

/* static */
void UINotificationMessage::cannotAddDiskEncryptionPassword(const CConsole &comConsole)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Bad password ..."),
        QApplication::translate("UIMessageCenter", "Bad password or authentication failure.") +
        UIErrorString::formatErrorInfo(comConsole),
        0);
}

/* static */
void UINotificationMessage::cannotAttachUSBDevice(const CConsole &comConsole, const QString &strDevice)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't attach USB device ..."),
        QApplication::translate("UIMessageCenter", "Failed to attach the USB device <b>%1</b> to the virtual machine <b>%2</b>.")
                                .arg(strDevice, CConsole(comConsole).GetMachine().GetName()) +
        UIErrorString::formatErrorInfo(comConsole));
}

/* static */
void UINotificationMessage::cannotAttachUSBDevice(const CVirtualBoxErrorInfo &comErrorInfo,
                                                  const QString &strDevice, const QString &strMachineName)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't attach USB device ..."),
        QApplication::translate("UIMessageCenter", "Failed to attach the USB device <b>%1</b> to the virtual machine <b>%2</b>.")
                                .arg(strDevice, strMachineName) +
        UIErrorString::formatErrorInfo(comErrorInfo));
}

/* static */
void UINotificationMessage::cannotDetachUSBDevice(const CConsole &comConsole, const QString &strDevice)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't detach USB device ..."),
        QApplication::translate("UIMessageCenter", "Failed to detach the USB device <b>%1</b> from the virtual machine <b>%2</b>.")
                                .arg(strDevice, CConsole(comConsole).GetMachine().GetName()) +
        UIErrorString::formatErrorInfo(comConsole));
}

/* static */
void UINotificationMessage::cannotDetachUSBDevice(const CVirtualBoxErrorInfo &comErrorInfo,
                                                  const QString &strDevice, const QString &strMachineName)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't detach USB device ..."),
        QApplication::translate("UIMessageCenter", "Failed to detach the USB device <b>%1</b> from the virtual machine <b>%2</b>.")
                                .arg(strDevice, strMachineName) +
        UIErrorString::formatErrorInfo(comErrorInfo));
}

/* static */
void UINotificationMessage::cannotAttachWebCam(const CEmulatedUSB &comDispatcher,
                                               const QString &strWebCamName, const QString &strMachineName)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't attach webcam ..."),
        QApplication::translate("UIMessageCenter", "Failed to attach the webcam <b>%1</b> to the virtual machine <b>%2</b>.")
                                .arg(strWebCamName, strMachineName) +
        UIErrorString::formatErrorInfo(comDispatcher));
}

/* static */
void UINotificationMessage::cannotDetachWebCam(const CEmulatedUSB &comDispatcher,
                                               const QString &strWebCamName, const QString &strMachineName)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't detach webcam ..."),
        QApplication::translate("UIMessageCenter", "Failed to detach the webcam <b>%1</b> from the virtual machine <b>%2</b>.")
                                .arg(strWebCamName, strMachineName) +
        UIErrorString::formatErrorInfo(comDispatcher));
}

/* static */
void UINotificationMessage::cannotSaveMachineSettings(const CMachine &comMachine, QWidget *pParent /* = 0 */)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't save machine settings ..."),
        QApplication::translate("UIMessageCenter", "Failed to save the settings of the virtual machine <b>%1</b> to "
                                                   "<b><nobr>%2</nobr></b>.")
                                                   .arg(CMachine(comMachine).GetName(),
                                                        CMachine(comMachine).GetSettingsFilePath()) +
        UIErrorString::formatErrorInfo(comMachine),
        pParent);
}

/* static */
void UINotificationMessage::cannotToggleAudioInput(const CAudioAdapter &comAdapter,
                                                   const QString &strMachineName, bool fEnable)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't toggle audio input ..."),
        (  fEnable
         ? QApplication::translate("UIMessageCenter", "Failed to enable the audio adapter input for the virtual machine <b>%1</b>.")
                                   .arg(strMachineName)
         : QApplication::translate("UIMessageCenter", "Failed to disable the audio adapter input for the virtual machine <b>%1</b>.")
                                   .arg(strMachineName)) +
        UIErrorString::formatErrorInfo(comAdapter));
}

/* static */
void UINotificationMessage::cannotToggleAudioOutput(const CAudioAdapter &comAdapter,
                                                    const QString &strMachineName, bool fEnable)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't toggle audio output ..."),
        (  fEnable
         ? QApplication::translate("UIMessageCenter", "Failed to enable the audio adapter output for the virtual machine <b>%1</b>.")
                                   .arg(strMachineName)
         : QApplication::translate("UIMessageCenter", "Failed to disable the audio adapter output for the virtual machine <b>%1</b>.")
                                   .arg(strMachineName)) +
        UIErrorString::formatErrorInfo(comAdapter));
}

/* static */
void UINotificationMessage::cannotToggleNetworkCable(const CNetworkAdapter &comAdapter,
                                                     const QString &strMachineName, bool fConnect)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't toggle network cable ..."),
        (  fConnect
         ? QApplication::translate("UIMessageCenter", "Failed to connect the network adapter cable of the virtual machine <b>%1</b>.")
                                   .arg(strMachineName)
         : QApplication::translate("UIMessageCenter", "Failed to disconnect the network adapter cable of the virtual machine <b>%1</b>.")
                                   .arg(strMachineName)) +
        UIErrorString::formatErrorInfo(comAdapter));
}

/* static */
void UINotificationMessage::cannotToggleRecording(const CRecordingSettings &comRecording, const QString &strMachineName, bool fEnable)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't toggle recording ..."),
        (  fEnable
         ? QApplication::translate("UIMessageCenter", "Failed to enable recording for the virtual machine <b>%1</b>.")
                                   .arg(strMachineName)
         : QApplication::translate("UIMessageCenter", "Failed to disable recording for the virtual machine <b>%1</b>.")
                                   .arg(strMachineName)) +
        UIErrorString::formatErrorInfo(comRecording));
}

/* static */
void UINotificationMessage::cannotToggleVRDEServer(const CVRDEServer &comServer,
                                                   const QString &strMachineName, bool fEnable)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't toggle VRDE server ..."),
        (  fEnable
         ? QApplication::translate("UIMessageCenter", "Failed to enable the remote desktop server for the virtual machine <b>%1</b>.")
                                   .arg(strMachineName)
         : QApplication::translate("UIMessageCenter", "Failed to disable the remote desktop server for the virtual machine <b>%1</b>.")
                                   .arg(strMachineName)) +
        UIErrorString::formatErrorInfo(comServer));
}

#ifdef VBOX_WITH_DRAG_AND_DROP
/* static */
void UINotificationMessage::cannotDropDataToGuest(const CDnDTarget &comDndTarget)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't drop data to guest ..."),
        QApplication::translate("UIMessageCenter", "Drag and drop operation from host to guest failed.") +
        UIErrorString::formatErrorInfo(comDndTarget));
}

/* static */
void UINotificationMessage::cannotDropDataToGuest(const CProgress &comProgress)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't drop data to guest ..."),
        QApplication::translate("UIMessageCenter", "Drag and drop operation from host to guest failed.") +
        UIErrorString::formatErrorInfo(comProgress));
}

/* static */
void UINotificationMessage::cannotDropDataToHost(const CDnDSource &comDnDSource)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't drop data to host ..."),
        QApplication::translate("UIMessageCenter", "Drag and drop operation from guest to host failed.") +
        UIErrorString::formatErrorInfo(comDnDSource));
}

/* static */
void UINotificationMessage::cannotDropDataToHost(const CProgress &comProgress)
{
    createMessage(
        QApplication::translate("UIMessageCenter", "Can't drop data to host ..."),
        QApplication::translate("UIMessageCenter", "Drag and drop operation from guest to host failed.") +
        UIErrorString::formatErrorInfo(comProgress));
}
#endif /* VBOX_WITH_DRAG_AND_DROP */

UINotificationMessage::UINotificationMessage(const QString &strName,
                                             const QString &strDetails,
                                             const QString &strInternalName,
                                             const QString &strHelpKeyword)
    : UINotificationSimple(strName,
                           strDetails,
                           strInternalName,
                           strHelpKeyword)
{
}

UINotificationMessage::~UINotificationMessage()
{
    /* Remove message from known: */
    m_messages.remove(internalName());
}

/* static */
void UINotificationMessage::createMessageInt(UINotificationCenter *pParent,
                                             const QString &strName,
                                             const QString &strDetails,
                                             const QString &strInternalName,
                                             const QString &strHelpKeyword)
{
    /* Make sure parent is set: */
    AssertPtr(pParent);
    UINotificationCenter *pEffectiveParent = pParent ? pParent : gpNotificationCenter;

    /* Check if message suppressed: */
    if (isSuppressed(strInternalName))
        return;

    /* Check if message already exists: */
    if (   !strInternalName.isEmpty()
        && m_messages.contains(strInternalName))
        return;

    /* Create message finally: */
    const QUuid uId = pEffectiveParent->append(new UINotificationMessage(strName,
                                                                         strDetails,
                                                                         strInternalName,
                                                                         strHelpKeyword));
    if (!strInternalName.isEmpty())
        m_messages[strInternalName] = uId;
}

/* static */
void UINotificationMessage::createBlockingMessageInt(UINotificationCenter *pParent,
                                                     const QString &strName,
                                                     const QString &strDetails,
                                                     const QString &strInternalName,
                                                     const QString &strHelpKeyword)
{
    /* Make sure parent is set: */
    AssertPtr(pParent);
    UINotificationCenter *pEffectiveParent = pParent ? pParent : gpNotificationCenter;

    /* Check if message suppressed: */
    if (isSuppressed(strInternalName))
        return;

    /* Create message finally: */
    QPointer<UINotificationMessage> pMessage = new UINotificationMessage(strName,
                                                                         strDetails,
                                                                         strInternalName,
                                                                         strHelpKeyword);
    pEffectiveParent->showBlocking(pMessage);
    delete pMessage;
}

/* static */
void UINotificationMessage::createMessage(const QString &strName,
                                          const QString &strDetails,
                                          QWidget *pParent /* = 0 */)
{
    /* Acquire notification-center, make sure it's present: */
    UINotificationCenter *pCenter = UINotificationCenter::acquire(pParent);
    AssertPtrReturnVoid(pCenter);

    /* Redirect to wrapper above: */
    return createMessageInt(pCenter, strName, strDetails, QString(), QString());
}

/* static */
void UINotificationMessage::createMessage(const QString &strName,
                                          const QString &strDetails,
                                          const QString &strInternalName,
                                          const QString &strHelpKeyword /* = QString() */,
                                          QWidget *pParent /* = 0 */)
{
    /* Acquire notification-center, make sure it's present: */
    UINotificationCenter *pCenter = UINotificationCenter::acquire(pParent);
    AssertPtrReturnVoid(pCenter);

    /* Redirect to wrapper above: */
    return createMessageInt(pCenter, strName, strDetails, strInternalName, strHelpKeyword);
}

/* static */
void UINotificationMessage::createBlockingMessage(const QString &strName,
                                                  const QString &strDetails,
                                                  QWidget *pParent /* = 0 */)
{
    /* Acquire notification-center, make sure it's present: */
    UINotificationCenter *pCenter = UINotificationCenter::acquire(pParent);
    AssertPtrReturnVoid(pCenter);

    /* Redirect to wrapper above: */
    return createBlockingMessageInt(pCenter, strName, strDetails, QString(), QString());
}

/* static */
void UINotificationMessage::createBlockingMessage(const QString &strName,
                                                  const QString &strDetails,
                                                  const QString &strInternalName,
                                                  const QString &strHelpKeyword /* = QString() */,
                                                  QWidget *pParent /* = 0 */)
{
    /* Acquire notification-center, make sure it's present: */
    UINotificationCenter *pCenter = UINotificationCenter::acquire(pParent);
    AssertPtrReturnVoid(pCenter);

    /* Redirect to wrapper above: */
    return createBlockingMessageInt(pCenter, strName, strDetails, strInternalName, strHelpKeyword);
}

/* static */
void UINotificationMessage::destroyMessage(const QString &strInternalName,
                                           UINotificationCenter *pParent /* = 0 */)
{
    /* Check if message really exists: */
    if (!m_messages.contains(strInternalName))
        return;

    /* Choose effective parent: */
    UINotificationCenter *pEffectiveParent = pParent ? pParent : gpNotificationCenter;

    /* Destroy message finally: */
    pEffectiveParent->revoke(m_messages.value(strInternalName));
    m_messages.remove(strInternalName);
}
