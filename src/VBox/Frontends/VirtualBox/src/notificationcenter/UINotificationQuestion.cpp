/* $Id: UINotificationQuestion.cpp 113265 2026-03-05 08:50:41Z sergey.dubov@oracle.com $ */
/** @file
 * VBox Qt GUI - Various UINotificationQuestion implementations.
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

/* GUI includes: */
#include "UIExtraDataManager.h"
#include "UIHostComboEditor.h"
#include "UIMedium.h"
#include "UINotificationCenter.h"
#include "UINotificationQuestion.h"
#include "UITranslator.h"

/* COM includes: */
#include "CMediumFormat.h"
#include "KMediumFormatCapabilities.h"


/* static */
QMap<QString, QUuid> UINotificationQuestion::m_questions = QMap<QString, QUuid>();

/* static */
bool UINotificationQuestion::confirmCheckingInaccessibleMedia()
{
    return createBlockingQuestion(
        QApplication::translate("UIMessageCenter", "Check inaccessible media?"),
        QApplication::translate("UIMessageCenter", "<p>One or more disk image files are not currently accessible. As a result, "
                                                   "you will not be able to operate virtual machines that use these files until "
                                                   "they become accessible later.</p><p>Press <b>Check</b> to open the Virtual "
                                                   "Media Manager window and see which files are inaccessible, or press "
                                                   "<b>Ignore</b> to ignore this message.</p>"),
        QStringList() << QApplication::translate("UIMessageCenter", "Ignore", "message") /* cancel button text */
                      << QApplication::translate("UIMessageCenter", "Check", "inaccessible media") /* ok button text */,
        true /* Ok by default? */,
        "confirmCheckingInaccessibleMedia" /* internal name */);
}

/* static */
bool UINotificationQuestion::confirmCreatingPath(const QString &strPath)
{
    return createBlockingQuestion(
        QApplication::translate("UIMessageCenter", "Create machine path?"),
        QApplication::translate("UIMessageCenter", "<p>Selected path doesn't exist:<br>%1</p>"
                                "<p>Would you like to create it?</p>").arg(strPath));
}

/* static */
bool UINotificationQuestion::confirmAutomaticCollisionResolve(const QString &strName, const QString &strGroupName)
{
    return createBlockingQuestion(
        QApplication::translate("UIMessageCenter", "Resolve name collision?"),
        QApplication::translate("UIMessageCenter", "<p>You are trying to move group <nobr><b>%1</b></nobr> to group "
                                                   "<nobr><b>%2</b></nobr> which already have another item with the same "
                                                   "name.</p><p>Would you like to automatically rename it?</p>")
                                                   .arg(strName, strGroupName),
        QStringList() << QString() /* cancel button text */
                      << QApplication::translate("UIMessageCenter", "Rename", "group") /* ok button text */);
}

/* static */
bool UINotificationQuestion::confirmMachineItemRemoval(const QString &strNames)
{
    return createBlockingQuestion(
        QApplication::translate("UIMessageCenter", "Remove machine items?"),
        QApplication::translate("UIMessageCenter", "<p>Remove these virtual machine items from the machine "
                                                   "list?</p><p><b>%1</b></p>").arg(strNames),
        QStringList() << QString() /* cancel button text */
                      << QApplication::translate("UIMessageCenter", "Remove", "machine item") /* ok button text */,
        false /* Ok by default? */);
}

/* static */
bool UINotificationQuestion::confirmSnapshotRemoval(const QString &strName)
{
    return createBlockingQuestion(
        QApplication::translate("UIMessageCenter", "Deleting the snapshot?"),
        QApplication::translate("UIMessageCenter", "<p>Deleting the snapshot will cause the state information saved in it to be "
                                                   "lost, and storage data spread over several image files that VirtualBox has "
                                                   "created together with the snapshot will be merged into one file. This can be "
                                                   "a lengthy process, and the information in the snapshot cannot be "
                                                   "recovered.</p></p>Are you sure you want to delete the selected snapshot "
                                                   "<b>%1</b>?</p>").arg(strName),
        QStringList() << QString() /* cancel button text */
                      << QApplication::translate("UIMessageCenter", "Delete", "snapshot") /* ok button text */);
}

/* static */
bool UINotificationQuestion::confirmStartMultipleMachines(const QString &strNames)
{
    return createBlockingQuestion(
        QApplication::translate("UIMessageCenter", "Start multiple machines?"),
        QApplication::translate("UIMessageCenter", "<p>You are about to start all of the following virtual machines:</p>"
                                                   "<p><b>%1</b></p><p>This could take some time and consume a lot of host "
                                                   "system resources. Do you wish to proceed?</p>").arg(strNames),
        QStringList() << QString() /* cancel button text */
                      << QApplication::translate("UIMessageCenter", "Start", "machine") /* ok button text */,
        true /* Ok by default? */,
        "confirmStartMultipleMachines" /* internal name */);
}

/* static */
bool UINotificationQuestion::confirmResetMachine(const QString &strNames)
{
    return createBlockingQuestion(
        QApplication::translate("UIMessageCenter", "Reset following machines?"),
        QApplication::translate("UIMessageCenter", "<p>Do you really want to reset the following virtual machines?</p>"
                                                   "<p><b>%1</b></p><p>This will cause any unsaved data in applications running "
                                                   "inside it to be lost.</p>").arg(strNames),
        QStringList() << QString() /* cancel button text */
                      << QApplication::translate("UIMessageCenter", "Reset", "machine") /* ok button text */,
        true /* Ok by default? */,
        "confirmResetMachine" /* internal name */);
}

/* static */
bool UINotificationQuestion::confirmACPIShutdownMachine(const QString &strNames)
{
    return createBlockingQuestion(
        QApplication::translate("UIMessageCenter", "Shutdown machine?"),
        QApplication::translate("UIMessageCenter", "<p>Shut down these VMs by sending the ACPI shutdown "
                                                   "signal?</p><p><b>%1</b></p>").arg(strNames),
        QStringList() << QString() /* cancel button text */
                      << QApplication::translate("UIMessageCenter", "Shut Down", "machine") /* ok button text */,
        true /* Ok by default? */,
        "confirmACPIShutdownMachine" /* internal name */);
}

/* static */
bool UINotificationQuestion::confirmPowerOffMachine(const QString &strNames)
{
    return createBlockingQuestion(
        QApplication::translate("UIMessageCenter", "Power off machine?"),
        QApplication::translate("UIMessageCenter", "<p>Close these VMs with no shutdown procedure?</p><p><b>%1</b></p><p>Unsaved "
                                                   "data in applications running on the VM will be lost.</p>").arg(strNames),
        QStringList() << QString() /* cancel button text */
                      << QApplication::translate("UIMessageCenter", "Power Off", "machine") /* ok button text */,
        true /* Ok by default? */,
        "confirmPowerOffMachine" /* internal name */);
}

/* static */
bool UINotificationQuestion::confirmDiscardSavedState(const QString &strNames)
{
    return createBlockingQuestion(
        QApplication::translate("UIMessageCenter", "Discard saved state?"),
        QApplication::translate("UIMessageCenter", "<p>Are you sure you want to discard the saved state of the following virtual "
                                                   "machines?</p><p><b>%1</b></p><p>This operation is equivalent to resetting or "
                                                   "powering off the machine without doing a proper shut down of the guest "
                                                   "OS.</p>").arg(strNames),
        QStringList() << QString() /* cancel button text */
                      << QApplication::translate("UIMessageCenter", "Discard", "saved state") /* ok button text */);
}

/* static */
bool UINotificationQuestion::confirmTerminateCloudInstance(const QString &strNames)
{
    return createBlockingQuestion(
        QApplication::translate("UIMessageCenter", "Terminate cloud instance?"),
        QApplication::translate("UIMessageCenter", "<p>Are you sure you want to terminate the cloud instance of the following "
                                                   "virtual machines?</p><p><b>%1</b></p>").arg(strNames),
        QStringList() << QString() /* cancel button text */
                      << QApplication::translate("UIMessageCenter", "Terminate", "cloud instance") /* ok button text */);
}

/* static */
bool UINotificationQuestion::confirmRemovingOfLastDVDDevice(QWidget *pParent)
{
    return createBlockingQuestion(
        QApplication::translate("UIMessageCenter", "Remove DVD device?"),
        QApplication::translate("UIMessageCenter", "<p>Are you sure you want to delete the optical drive?</p><p>You will not "
                                                   "be able to insert any optical disks or ISO images or install the Guest "
                                                   "Additions without it!</p>"),
        QStringList() << QString() /* cancel button text */
                      << QApplication::translate("UIMessageCenter", "Remove", "device") /* ok button text */,
        false /* ok button by default? */,
        QString() /* internal name */,
        QString() /* help keyword */,
        pParent);
}

/* static */
bool UINotificationQuestion::confirmStorageBusChangeWithOpticalRemoval(QWidget *pParent)
{
    return createBlockingQuestion(
        QApplication::translate("UIMessageCenter", "Confirm storage bus change?"),
        QApplication::translate("UIMessageCenter", "<p>This controller has optical devices attached.  You have requested storage "
                                                   "bus change to type which doesn't support optical devices.</p><p>If you "
                                                   "proceed optical devices will be removed.</p>"),
        QStringList() /* no button name redefinition */,
        true /* ok button by default? */,
        QString() /* internal name */,
        QString() /* help keyword */,
        pParent);
}

/* static */
bool UINotificationQuestion::confirmStorageBusChangeWithExcessiveRemoval(QWidget *pParent)
{
    return createBlockingQuestion(
        QApplication::translate("UIMessageCenter", "Confirm storage bus change?"),
        QApplication::translate("UIMessageCenter", "<p>This controller has devices attached.  You have requested storage bus "
                                                   "change to type which supports smaller amount of attached devices.</p><p>If "
                                                   "you proceed excessive devices will be removed.</p>"),
        QStringList() /* no button name redefinition */,
        true /* ok button by default? */,
        QString() /* internal name */,
        QString() /* help keyword */,
        pParent);
}

/* static */
bool UINotificationQuestion::confirmCancelingPortForwardingDialog(QWidget *pParent)
{
    return createBlockingQuestion(
        QApplication::translate("UIMessageCenter", "Discard port forwarding changes?"),
        QApplication::translate("UIMessageCenter", "<p>There are unsaved changes in the port forwarding configuration.</p>"
                                                   "<p>If you proceed your changes will be discarded.</p>"),
        QStringList() /* no button name redefinition */,
        false /* ok button by default? */,
        QString() /* internal name */,
        QString() /* help keyword */,
        pParent);
}

/* static */
bool UINotificationQuestion::confirmRestoringDefaultKeys(QWidget *pParent)
{
    return createBlockingQuestion(
        QApplication::translate("UIMessageCenter", "Restore default keys?"),
        QApplication::translate("UIMessageCenter", "<p>You are going to restore default secure boot keys.</p>"
                                                   "<p>If you proceed your current keys will be rewritten. "
                                                   "You may not be able to boot affected VM anymore.</p>"),
        QStringList() /* no button name redefinition */,
        false /* ok button by default? */,
        QString() /* internal name */,
        QString() /* help keyword */,
        pParent);
}

/* static */
bool UINotificationQuestion::confirmInstallExtensionPack(const QString &strPackName,
                                                         const QString &strPackVersion,
                                                         const QString &strPackDescription,
                                                         QWidget *pParent)
{
    return createBlockingQuestion(
        QApplication::translate("UIMessageCenter", "Install extension pack?"),
        QApplication::translate("UIMessageCenter", "<p>You are about to install a VirtualBox extension pack. Extension packs "
                                                   "complement the functionality of VirtualBox and can contain system level "
                                                   "software that could be potentially harmful to your system. Please review "
                                                   "the description below and only proceed if you have obtained the extension "
                                                   "pack from a trusted source.</p>"
                                                   "<p><table cellpadding=0 cellspacing=5>"
                                                   "<tr><td><b>Name:&nbsp;&nbsp;</b></td><td>%1</td></tr>"
                                                   "<tr><td><b>Version:&nbsp;&nbsp;</b></td><td>%2</td></tr>"
                                                   "<tr><td><b>Description:&nbsp;&nbsp;</b></td><td>%3</td></tr>"
                                                   "</table></p>")
                                                   .arg(strPackName).arg(strPackVersion).arg(strPackDescription),
        QStringList() << QString() /* cancel button text */
                      << QApplication::translate("UIMessageCenter", "Install", "extension pack") /* ok button text */,
        true /* ok button by default? */,
        QString() /* internal name */,
        QString() /* help keyword */,
        pParent);
}

/* static */
bool UINotificationQuestion::confirmReplaceExtensionPack(const QString &strPackName,
                                                         const QString &strPackVersionNew,
                                                         const QString &strPackVersionOld,
                                                         const QString &strPackDescription,
                                                         QWidget *pParent)
{
    /* Prepare initial message: */
    const QString strTitle =
        QApplication::translate("UIMessageCenter", "Replace extension pack?");
    const QString strBelehrung =
        QApplication::translate("UIMessageCenter", "Extension packs complement the functionality of VirtualBox and can contain "
                                                   "system level software that could be potentially harmful to your system. "
                                                   "Please review the description below and only proceed if you have obtained "
                                                   "the extension pack from a trusted source.");

    /* Compare versions: */
    QByteArray ba1     = strPackVersionNew.toUtf8();
    QByteArray ba2     = strPackVersionOld.toUtf8();
    int        iVerCmp = RTStrVersionCompare(ba1.constData(), ba2.constData());

    /* Show the question: */
    bool fRc;
    if (iVerCmp > 0)
        fRc = createBlockingQuestion(
            strTitle,
            QApplication::translate("UIMessageCenter", "<p>An older version of the extension pack is already installed, would "
                                                       "you like to upgrade? "
                                                       "<p>%1</p>"
                                                       "<p><table cellpadding=0 cellspacing=5>"
                                                       "<tr><td><b>Name:&nbsp;&nbsp;</b></td><td>%2</td></tr>"
                                                       "<tr><td><b>New Version:&nbsp;&nbsp;</b></td><td>%3</td></tr>"
                                                       "<tr><td><b>Current Version:&nbsp;&nbsp;</b></td><td>%4</td></tr>"
                                                       "<tr><td><b>Description:&nbsp;&nbsp;</b></td><td>%5</td></tr>"
                                                       "</table></p>")
                                                       .arg(strBelehrung).arg(strPackName).arg(strPackVersionNew)
                                                       .arg(strPackVersionOld).arg(strPackDescription),
            QStringList() << QString() /* cancel button text */
                          << QApplication::translate("UIMessageCenter", "Upgrade", "extension pack") /* ok button text */,
            true /* ok button by default? */,
            QString() /* internal name */,
            QString() /* help keyword */,
            pParent);
    else if (iVerCmp < 0)
        fRc = createBlockingQuestion(
            strTitle,
            QApplication::translate("UIMessageCenter", "<p>An newer version of the extension pack is already installed, would "
                                                       "you like to downgrade? "
                                                       "<p>%1</p>"
                                                       "<p><table cellpadding=0 cellspacing=5>"
                                                       "<tr><td><b>Name:&nbsp;&nbsp;</b></td><td>%2</td></tr>"
                                                       "<tr><td><b>New Version:&nbsp;&nbsp;</b></td><td>%3</td></tr>"
                                                       "<tr><td><b>Current Version:&nbsp;&nbsp;</b></td><td>%4</td></tr>"
                                                       "<tr><td><b>Description:&nbsp;&nbsp;</b></td><td>%5</td></tr>"
                                                       "</table></p>")
                                                       .arg(strBelehrung).arg(strPackName).arg(strPackVersionNew)
                                                       .arg(strPackVersionOld).arg(strPackDescription),
            QStringList() << QString() /* cancel button text */
                          << QApplication::translate("UIMessageCenter", "Downgrade", "extension pack") /* ok button text */,
            true /* ok button by default? */,
            QString() /* internal name */,
            QString() /* help keyword */,
            pParent);
    else
        fRc = createBlockingQuestion(
            strTitle,
            QApplication::translate("UIMessageCenter", "<p>The extension pack is already installed with the same version, would "
                                                       "you like reinstall it? "
                                                       "<p>%1</p>"
                                                       "<p><table cellpadding=0 cellspacing=5>"
                                                       "<tr><td><b>Name:&nbsp;&nbsp;</b></td><td>%2</td></tr>"
                                                       "<tr><td><b>Version:&nbsp;&nbsp;</b></td><td>%3</td></tr>"
                                                       "<tr><td><b>Description:&nbsp;&nbsp;</b></td><td>%4</td></tr>"
                                                       "</table></p>")
                                                       .arg(strBelehrung).arg(strPackName).arg(strPackVersionOld)
                                                       .arg(strPackDescription),
            QStringList() << QString() /* cancel button text */
                          << QApplication::translate("UIMessageCenter", "Reinstall", "extension pack") /* ok button text */,
            true /* ok button by default? */,
            QString() /* internal name */,
            QString() /* help keyword */,
            pParent);
    return fRc;
}

/* static */
bool UINotificationQuestion::confirmRemoveExtensionPack(const QString &strPackName,
                                                        QWidget *pParent)
{
    return createBlockingQuestion(
        QApplication::translate("UIMessageCenter", "Remove extension pack?"),
        QApplication::translate("UIMessageCenter", "<p>You are about to remove the VirtualBox extension pack <b>%1</b>.</p>"
                                                   "<p>Are you sure you want to proceed?</p>").arg(strPackName),
        QStringList() << QString() /* cancel button text */
                      << QApplication::translate("UIMessageCenter", "Remove", "extension pack") /* ok button text */,
        false /* ok button by default? */,
        QString() /* internal name */,
        QString() /* help keyword */,
        pParent);
}

/* static */
bool UINotificationQuestion::confirmMediumRelease(const UIMedium &guiMedium,
                                                  bool fInduced,
                                                  const QStringList &usage,
                                                  QWidget *pParent)
{
    /* Show the question: */
    return !fInduced
           ? createBlockingQuestion(
                QApplication::translate("UIMessageCenter", "Release disk image?"),
                QApplication::translate("UIMessageCenter", "<p>Are you sure you want to release the disk image file "
                                                           "<nobr><b>%1</b></nobr>?</p><p>This will detach it from the following "
                                                           "virtual machine(s): <b>%2</b>.</p>")
                                                           .arg(guiMedium.location(), usage.join(", ")),
                QStringList() << QString() /* cancel button text */
                              << QApplication::translate("UIMessageCenter", "Release", "disk image file") /* ok button text */,
                true /* ok button by default? */,
                QString() /* internal name */,
                QString() /* help keyword */,
                pParent)
           : createBlockingQuestion(
                QApplication::translate("UIMessageCenter", "Release disk image?"),
                QApplication::translate("UIMessageCenter", "<p>The changes you requested require this disk to be released from "
                                                           "the machines it is attached to.</p><p>Are you sure you want to "
                                                           "release the disk image file <nobr><b>%1</b></nobr>?</p><p>This will "
                                                           "detach it from the following virtual machine(s): <b>%2</b>.</p>")
                                                           .arg(guiMedium.location(), usage.join(", ")),
                QStringList() << QString() /* cancel button text */
                              << QApplication::translate("UIMessageCenter", "Release", "disk image file") /* ok button text */,
                true /* ok button by default? */,
                QString() /* internal name */,
                QString() /* help keyword */,
                pParent);
}

/* static */
bool UINotificationQuestion::confirmMediumRemoval(const UIMedium &guiMedium,
                                                  QWidget *pParent)
{
    /* Prepare the message: */
    QString strHeader;
    QString strMessage;
    switch (guiMedium.type())
    {
        case UIMediumDeviceType_HardDisk:
        {
            strHeader = QApplication::translate("UIMessageCenter", "Remove hard disk?");
            strMessage = QApplication::translate("UIMessageCenter", "<p>Are you sure you want to remove the virtual hard disk "
                                                                    "<nobr><b>%1</b></nobr> from the list of known disk image "
                                                                    "files?</p>");
            /* Compose capabilities flag: */
            qulonglong caps = 0;
            QVector<KMediumFormatCapabilities> capabilities;
            capabilities = guiMedium.medium().GetMediumFormat().GetCapabilities();
            for (int i = 0; i < capabilities.size(); ++i)
                caps |= capabilities[i];
            /* Check capabilities for additional options: */
            if (caps & KMediumFormatCapabilities_File)
            {
                if (guiMedium.state() == KMediumState_Inaccessible)
                    strMessage += QApplication::translate("UIMessageCenter", "<p>As this hard disk is inaccessible its image "
                                                                             "file cannot be deleted.</p>");
            }
            break;
        }
        case UIMediumDeviceType_DVD:
        {
            strHeader = QApplication::translate("UIMessageCenter", "Remove optical disk?");
            strMessage = QApplication::translate("UIMessageCenter", "<p>Are you sure you want to remove the virtual optical disk "
                                                                    "<nobr><b>%1</b></nobr> from the list of known disk image "
                                                                    "files?</p>");
            strMessage += QApplication::translate("UIMessageCenter", "<p>Note that the storage unit of this medium will not be "
                                                                     "deleted and that it will be possible to use it later "
                                                                     "again.</p>");
            break;
        }
        case UIMediumDeviceType_Floppy:
        {
            strHeader = QApplication::translate("UIMessageCenter", "Remove floppy disk?");
            strMessage = QApplication::translate("UIMessageCenter", "<p>Are you sure you want to remove the virtual floppy disk "
                                                                    "<nobr><b>%1</b></nobr> from the list of known disk image "
                                                                    "files?</p>");
            strMessage += QApplication::translate("UIMessageCenter", "<p>Note that the storage unit of this medium will not be "
                                                                     "deleted and that it will be possible to use it later "
                                                                     "again.</p>");
            break;
        }
        default:
            break;
    }
    /* Show the question: */
    return createBlockingQuestion(
                strHeader,
                strMessage.arg(guiMedium.location()),
                QStringList() << QString() /* cancel button text */
                              << QApplication::translate("UIMessageCenter", "Remove", "disk image file") /* ok button text */,
                true /* ok button by default? */,
                QString() /* internal name */,
                QString() /* help keyword */,
                pParent);
}

/* static */
int UINotificationQuestion::confirmDeleteHardDiskStorage(const QString &strLocation,
                                                         QWidget *pParent)
{
    return createBlockingQuestion(
        QApplication::translate("UIMessageCenter", "Delete hard disk?"),
        QApplication::translate("UIMessageCenter", "<p>Do you want to delete the storage unit of the virtual hard disk "
                                                   "<nobr><b>%1</b></nobr>?</p><p>If you select <b>Delete</b> then the specified "
                                                   "storage unit will be permanently deleted. This operation <b>cannot be "
                                                   "undone</b>.</p><p>If you select <b>Keep</b> then the hard disk will be only "
                                                   "removed from the list of known hard disks, but the storage unit will be left "
                                                   "untouched which makes it possible to add this hard disk to the list later "
                                                   "again.</p>").arg(strLocation),
        QStringList() << QString() /* cancel button text */
                      << QApplication::translate("UIMessageCenter", "Delete", "hard disk storage") /* ok button text */
                      << QApplication::translate("UIMessageCenter", "Keep", "hard disk storage") /* yes button text */,
        false /* ok button by default? */,
        QString() /* internal name */,
        QString() /* help keyword */,
        pParent);
}

/* static */
bool UINotificationQuestion::confirmInaccesibleMediaClear(const QStringList &media,
                                                          UIMediumDeviceType enmType,
                                                          QWidget *pParent)
{
    if (media.isEmpty())
        return false;

    if (enmType != UIMediumDeviceType_DVD && enmType != UIMediumDeviceType_Floppy)
        return false;

    QString strDetails("<!--EOM-->");
    QString strDetailMessage;

    if (enmType == UIMediumDeviceType_DVD)
        strDetailMessage = QApplication::translate("UIMessageCenter", "The list of inaccessible DVDs is as follows:");
    else
        strDetailMessage = QApplication::translate("UIMessageCenter", "The list of inaccessible floppy disks is as follows:");

    if (!strDetailMessage.isEmpty())
        strDetails.prepend(QString("<p>%1</p>").arg(UITranslator::emphasize(strDetailMessage)));

    strDetails += QString("<table bgcolor=%1 border=0 cellspacing=5 cellpadding=0 width=100%>")
                         .arg(QApplication::palette().color(QPalette::Active, QPalette::Window).name(QColor::HexRgb));
    foreach (const QString &strDVD, media)
        strDetails += QString("<tr><td>%1</td></tr>").arg(strDVD);
    strDetails += QString("</table>");

    if (!strDetails.isEmpty())
        strDetails = "<qt>" + strDetails + "</qt>";

    if (enmType == UIMediumDeviceType_DVD)
        return createBlockingQuestion(
            QApplication::translate("UIMessageCenter", "Clear inaccessible media?"),
            QApplication::translate("UIMessageCenter", "<p>This will clear the optical disk list by releasing inaccessible DVDs "
                                                       "from the virtual machines they are attached to and removing them from "
                                                       "the list of registered media.<p>Are you sure?") + strDetails,
            QStringList() << QString() /* cancel button text */
                          << QApplication::translate("UIMessageCenter", "Clear", "inaccessible media") /* ok button text */,
            false /* ok button by default? */,
            QString() /* internal name */,
            QString() /* help keyword */,
            pParent);
    else
        return createBlockingQuestion(
            QApplication::translate("UIMessageCenter", "Clear inaccessible media?"),
            QApplication::translate("UIMessageCenter", "<p>This will clear the floppy disk list by releasing inaccessible disks "
                                                       "from the virtual machines they are attached to and removing them from "
                                                       "the list of registered media.<p>Are you sure?") + strDetails,
            QStringList() << QString() /* cancel button text */
                          << QApplication::translate("UIMessageCenter", "Clear", "inaccessible media") /* ok button text */,
            false /* ok button by default? */,
            QString() /* internal name */,
            QString() /* help keyword */,
            pParent);
}

/* static */
bool UINotificationQuestion::confirmCloudNetworkRemoval(const QString &strName, QWidget *pParent)
{
    return createBlockingQuestion(
        QApplication::translate("UIMessageCenter", "Remove cloud network?"),
        QApplication::translate("UIMessageCenter", "<p>Do you want to remove the cloud network <nobr><b>%1</b>?</nobr></p>"
                                                   "<p>If this network is in use by one or more virtual machine network adapters "
                                                   "these adapters will no longer be usable until you correct their settings by "
                                                   "either choosing a different network name or a different adapter attachment "
                                                   "type.</p>").arg(strName),
        QStringList() << QString() /* cancel button text */
                      << QApplication::translate("UIMessageCenter", "Remove", "network") /* ok button text */,
        false /* ok button by default? */,
        QString() /* internal name */,
        QString() /* help keyword */,
        pParent);
}

/* static */
bool UINotificationQuestion::confirmHostNetworkInterfaceRemoval(const QString &strName, QWidget *pParent)
{
    return createBlockingQuestion(
        QApplication::translate("UIMessageCenter", "Remove host-only network interface?"),
        QApplication::translate("UIMessageCenter", "<p>Deleting this host-only network will remove the host-only interface this "
                                                   "network is based on. Do you want to remove the (host-only network) interface "
                                                   "<nobr><b>%1</b>?</nobr></p><p><b>Note:</b> this interface may be in use by "
                                                   "one or more virtual network adapters belonging to one of your VMs. After it "
                                                   "is removed, these adapters will no longer be usable until you correct their "
                                                   "settings by either choosing a different interface name or a different "
                                                   "adapter attachment type.</p>").arg(strName),
        QStringList() << QString() /* cancel button text */
                      << QApplication::translate("UIMessageCenter", "Remove", "interface") /* ok button text */,
        false /* ok button by default? */,
        QString() /* internal name */,
        QString() /* help keyword */,
        pParent);
}

/* static */
bool UINotificationQuestion::confirmHostOnlyNetworkRemoval(const QString &strName, QWidget *pParent)
{
    return createBlockingQuestion(
        QApplication::translate("UIMessageCenter", "Remove host-only network?"),
        QApplication::translate("UIMessageCenter", "<p>Do you want to remove the host-only network <nobr><b>%1</b>?</nobr></p>"
                                                   "<p>If this network is in use by one or more virtual machine network adapters "
                                                   "these adapters will no longer be usable until you correct their settings by "
                                                   "either choosing a different network name or a different adapter attachment "
                                                   "type.</p>").arg(strName),
        QStringList() << QString() /* cancel button text */
                      << QApplication::translate("UIMessageCenter", "Remove", "network") /* ok button text */,
        false /* ok button by default? */,
        QString() /* internal name */,
        QString() /* help keyword */,
        pParent);
}

/* static */
bool UINotificationQuestion::confirmNATNetworkRemoval(const QString &strName, QWidget *pParent)
{
    return createBlockingQuestion(
        QApplication::translate("UIMessageCenter", "Remove NAT network?"),
        QApplication::translate("UIMessageCenter", "<p>Do you want to remove the NAT network <nobr><b>%1</b>?</nobr></p><p>If "
                                                   "this network is in use by one or more virtual machine network adapters these "
                                                   "adapters will no longer be usable until you correct their settings by either "
                                                   "choosing a different network name or a different adapter attachment "
                                           "type.</p>").arg(strName),
        QStringList() << QString() /* cancel button text */
                      << QApplication::translate("UIMessageCenter", "Remove", "network") /* ok button text */,
        false /* ok button by default? */,
        QString() /* internal name */,
        QString() /* help keyword */,
        pParent);
}

/* static */
bool UINotificationQuestion::confirmCloudProfileRemoval(const QString &strName, QWidget *pParent)
{
    return createBlockingQuestion(
        QApplication::translate("UIMessageCenter", "Remove cloud profile?"),
        QApplication::translate("UIMessageCenter", "<p>Do you want to remove the cloud profile <nobr><b>%1</b>?</nobr></p>")
                                                   .arg(strName),
        QStringList() << QString() /* cancel button text */
                      << QApplication::translate("UIMessageCenter", "Remove", "profile") /* ok button text */,
        false /* ok button by default? */,
        QString() /* internal name */,
        QString() /* help keyword */,
        pParent);
}

/* static */
bool UINotificationQuestion::confirmCloudProfilesImport(QWidget *pParent)
{
    return createBlockingQuestion(
        QApplication::translate("UIMessageCenter", "Import cloud profiles?"),
        QApplication::translate("UIMessageCenter", "<p>Do you want to import cloud profiles from external files?</p>"
                                                   "<p>VirtualBox cloud profiles will be overwritten and their data will be "
                                                   "lost.</p>"),
        QStringList() << QString() /* cancel button text */
                      << QApplication::translate("UIMessageCenter", "Import", "profiles") /* ok button text */,
        false /* ok button by default? */,
        QString() /* internal name */,
        QString() /* help keyword */,
        pParent);
}

/* static */
bool UINotificationQuestion::confirmCloudConsoleApplicationRemoval(const QString &strName)
{
    return createBlockingQuestion(
        QApplication::translate("UIMessageCenter", "Remove cloud console application?"),
        QApplication::translate("UIMessageCenter", "<p>Do you want to remove the cloud console application "
                                                   "<nobr><b>%1</b>?</nobr></p>").arg(strName),
        QStringList() << QString() /* cancel button text */
                      << QApplication::translate("UIMessageCenter", "Remove", "application") /* ok button text */,
        false /* ok button by default? */);
}

/* static */
bool UINotificationQuestion::confirmCloudConsoleProfileRemoval(const QString &strName)
{
    return createBlockingQuestion(
        QApplication::translate("UIMessageCenter", "Remove cloud console application?"),
        QApplication::translate("UIMessageCenter", "<p>Do you want to remove the cloud console profile "
                                                   "<nobr><b>%1</b>?</nobr></p>").arg(strName),
        QStringList() << QString() /* cancel button text */
                      << QApplication::translate("UIMessageCenter", "Remove", "profile") /* ok button text */,
        false /* ok button by default? */);
}

#ifdef VBOX_GUI_WITH_NETWORK_MANAGER
/* static */
bool UINotificationQuestion::confirmLookingForGuestAdditions()
{
    return createBlockingQuestion(
        QApplication::translate("UIMessageCenter", "Look for guest additions?"),
        QApplication::translate("UIMessageCenter", "<p>Could not find the <b>VirtualBox Guest Additions</b> disk image file.</p>"
                                                   "<p>Do you wish to download this disk image file from the Internet?</p>"));
}

/* static */
bool UINotificationQuestion::confirmDownloadingGuestAdditions(const QString &strUrl, qulonglong uSize)
{
    return createBlockingQuestion(
        QApplication::translate("UIMessageCenter", "Download guest additions?"),
        QApplication::translate("UIMessageCenter", "<p>Are you sure you want to download the <b>VirtualBox Guest Additions</b> "
                                                   "disk image file from <nobr><a href=\"%1\">%1</a></nobr> (size %2 "
                                                   "bytes)?</p>")
                                                   .arg(strUrl, QLocale(UITranslator::languageId()).toString(uSize)),
        QStringList() << QString() /* cancel button text */
                      << QApplication::translate("UIMessageCenter", "Download", "guest additions") /* ok button text */);
}

/* static */
bool UINotificationQuestion::confirmMountingGuestAdditions(const QString &strUrl, const QString &strSrc)
{
    return createBlockingQuestion(
        QApplication::translate("UIMessageCenter", "Mount guest additions?"),
        QApplication::translate("UIMessageCenter", "<p>The <b>VirtualBox Guest Additions</b> disk image file has been "
                                                   "successfully downloaded from <nobr><a href=\"%1\">%1</a></nobr> and saved "
                                                   "locally as <nobr><b>%2</b>.</nobr></p><p>Do you wish to continue with Guest "
                                                   "Additions installation?</p>").arg(strUrl, strSrc),
        QStringList() << QString() /* cancel button text */
                      << QApplication::translate("UIMessageCenter", "Mount", "guest additions") /* ok button text */);
}

/* static */
bool UINotificationQuestion::confirmLookingForExtensionPack(const QString &strExtPackName, const QString &strExtPackVersion)
{
    return createBlockingQuestion(
        QApplication::translate("UIMessageCenter", "Look for extension pack?"),
        QApplication::translate("UIMessageCenter", "<p>You have an old version (%1) of the <b><nobr>%2</nobr></b> installed.</p>"
                                                   "<p>Do you wish to download latest one from the Internet?</p>")
                                                   .arg(strExtPackVersion).arg(strExtPackName));
}

/* static */
bool UINotificationQuestion::confirmDownloadingExtensionPack(const QString &strExtPackName, const QString &strURL, qulonglong uSize)
{
    return createBlockingQuestion(
        QApplication::translate("UIMessageCenter", "Download extension pack?"),
        QApplication::translate("UIMessageCenter", "<p>Are you sure you want to download the <b><nobr>%1</nobr></b> from "
                                                   "<nobr><a href=\"%2\">%2</a></nobr> (size %3 bytes)?</p>")
                                                   .arg(strExtPackName, strURL,
                                                        QLocale(UITranslator::languageId()).toString(uSize)),
        QStringList() << QString() /* cancel button text */
                      << QApplication::translate("UIMessageCenter", "Download", "extension pack") /* ok button text */);
}

/* static */
bool UINotificationQuestion::confirmInstallingExtentionPack(const QString &strExtPackName, const QString &strFrom, const QString &strTo)
{
    return createBlockingQuestion(
        QApplication::translate("UIMessageCenter", "Install extension pack?"),
        QApplication::translate("UIMessageCenter", "<p>The <b><nobr>%1</nobr></b> has been successfully downloaded from "
                                                   "<nobr><a href=\"%2\">%2</a></nobr> and saved locally as "
                                                   "<nobr><b>%3</b>.</nobr></p><p>Do you wish to install this extension "
                                                   "pack?</p>").arg(strExtPackName, strFrom, strTo),
        QStringList() << QString() /* cancel button text */
                      << QApplication::translate("UIMessageCenter", "Install", "extension pack") /* ok button text */);
}

/* static */
bool UINotificationQuestion::confirmDeletingExtentionPackFile(const QString &strTo)
{
    return createBlockingQuestion(
        QApplication::translate("UIMessageCenter", "Delete extension pack file?"),
        QApplication::translate("UIMessageCenter", "Do you want to delete the downloaded file "
                                                   "<nobr><b>%1</b></nobr>?").arg(strTo),
        QStringList() << QString() /* cancel button text */
                      << QApplication::translate("UIMessageCenter", "Delete", "extension pack") /* ok button text */);
}

/* static */
bool UINotificationQuestion::confirmDeletingOldExtentionPackFiles(const QStringList &strFiles)
{
    return createBlockingQuestion(
        QApplication::translate("UIMessageCenter", "Delete extension pack files?"),
        QApplication::translate("UIMessageCenter", "Do you want to delete following list of files "
                                                   "<nobr><b>%1</b></nobr>?").arg(strFiles.join(",")),
        QStringList() << QString() /* cancel button text */
                      << QApplication::translate("UIMessageCenter", "Delete", "extension pack") /* ok button text */);
}
#endif /* VBOX_GUI_WITH_NETWORK_MANAGER */

/* static */
bool UINotificationQuestion::warnAboutNetworkInterfaceNotFound(const QString &strMachineName, const QString &strIfNames)
{
    return createBlockingQuestion(
        QApplication::translate("UIMessageCenter", "Open network configuration?"),
        QApplication::translate("UIMessageCenter", "<p>Could not start the machine <b>%1</b> because the following physical "
                                                   "network interfaces were not found:</p><p><b>%2</b></p><p>You can either "
                                                   "change the machine's network settings or stop the machine.</p>")
                                                   .arg(strMachineName, strIfNames),
        QStringList() << QApplication::translate("UIMessageCenter", "Close VM") /* cancel button text */
                      << QApplication::translate("UIMessageCenter", "Change Network Settings") /* ok button text */);
}

/* static */
bool UINotificationQuestion::confirmInputCapture(bool &fAutoConfirmed)
{
    /* Will the question be auto-confirmed? */
    fAutoConfirmed = isSuppressed("confirmInputCapture");

    /* Now the question itself: */
    return createBlockingQuestion(
        QApplication::translate("UIMessageCenter", "Capture input?"),
        QApplication::translate("UIMessageCenter", "<p>You have <b>clicked the mouse</b> inside the Virtual Machine display or "
                                                   "pressed the <b>host key combo</b>. This will cause the Virtual Machine to "
                                                   "<b>capture</b> the host mouse pointer (only if the mouse pointer integration "
                                                   "is not currently supported by the guest OS) and the keyboard, which will "
                                                   "make them unavailable to other applications running on your host machine.</p>"
                                                   "<p>You can press the <b>host key combo</b> at any time to <b>uncapture</b> "
                                                   "the keyboard and mouse (if it is captured) and return them to normal "
                                                   "operation. The currently assigned host key combo is shown on the status bar "
                                                   "at the bottom of the Virtual Machine window, next to "
                                                   "the&nbsp;<img src=:/hostkey_16px.png/>&nbsp;icon. This icon, together with "
                                                   "the mouse icon placed nearby, indicate the current keyboard and mouse "
                                                   "capture state.</p>") +
        QApplication::translate("UIMessageCenter", "<p>The host key combo is currently defined as <b>%1</b>.</p>",
                                "additional message box paragraph")
                                .arg(UIHostCombo::toReadableString(gEDataManager->hostKeyCombination())),
        QStringList() << QString() /* cancel button text */
                      << QApplication::translate("UIMessageCenter", "Capture", "do input capture") /* ok button text */,
        false /* ok button by default? */,
        "confirmInputCapture" /* internal name */);
}

/* static */
bool UINotificationQuestion::confirmGoingFullscreen(const QString &strHotKey)
{
    return createBlockingQuestion(
        QApplication::translate("UIMessageCenter", "Switch to full-screen mode?"),
        QApplication::translate("UIMessageCenter", "<p>The virtual machine window will be now switched to <b>full-screen</b> "
                                                   "mode. You can go back to windowed mode at any time by pressing <b>%1</b>.</p>"
                                                   "<p>Note that the <i>Host</i> key is currently defined as <b>%2</b>.</p>"
                                                   "<p>Note that the main menu bar is hidden in full-screen mode. You can access "
                                                   "it by pressing <b>Host+Home</b>.</p>")
                                                   .arg(strHotKey,
                                                        UIHostCombo::toReadableString(gEDataManager->hostKeyCombination())),
        QStringList() << QString() /* cancel button text */
                      << QApplication::translate("UIMessageCenter", "Switch") /* ok button text */,
        true /* ok button by default? */,
        "confirmGoingFullscreen");
}

/* static */
bool UINotificationQuestion::confirmGoingSeamless(const QString &strHotKey)
{
    return createBlockingQuestion(
        QApplication::translate("UIMessageCenter", "Switch to seamless mode?"),
        QApplication::translate("UIMessageCenter", "<p>The virtual machine window will be now switched to <b>Seamless</b> mode. "
                                                   "You can go back to windowed mode at any time by pressing <b>%1</b>.</p>"
                                                   "<p>Note that the <i>Host</i> key is currently defined as <b>%2</b>.</p>"
                                                   "<p>Note that the main menu bar is hidden in seamless mode. You can access it "
                                                   "by pressing <b>Host+Home</b>.</p>")
                                                   .arg(strHotKey,
                                                        UIHostCombo::toReadableString(gEDataManager->hostKeyCombination())),
        QStringList() << QString() /* cancel button text */
                      << QApplication::translate("UIMessageCenter", "Switch") /* ok button text */,
        true /* ok button by default? */,
        "confirmGoingSeamless");
}

/* static */
bool UINotificationQuestion::confirmGoingScale(const QString &strHotKey)
{
    return createBlockingQuestion(
        QApplication::translate("UIMessageCenter", "Switch to scale mode?"),
        QApplication::translate("UIMessageCenter", "<p>The virtual machine window will be now switched to <b>Scale</b> mode. "
                                                   "You can go back to windowed mode at any time by pressing <b>%1</b>.</p>"
                                                   "<p>Note that the <i>Host</i> key is currently defined as <b>%2</b>.</p>"
                                                   "<p>Note that the main menu bar is hidden in scaled mode. You can access it "
                                                   "by pressing <b>Host+Home</b>.</p>")
                                                   .arg(strHotKey,
                                                        UIHostCombo::toReadableString(gEDataManager->hostKeyCombination())),
        QStringList() << QString() /* cancel button text */
                      << QApplication::translate("UIMessageCenter", "Switch") /* ok button text */,
        true /* ok button by default? */,
        "confirmGoingScale");
}

/* static */
bool UINotificationQuestion::confirmGoingFullscreenAnyway(quint64 uMinVRAM)
{
    return createBlockingQuestion(
        QApplication::translate("UIMessageCenter", "Still switch to full-screen mode?"),
        QApplication::translate("UIMessageCenter", "<p>Could not switch the guest display to full-screen mode due to "
                                                   "insufficient guest video memory.</p><p>You should configure the virtual "
                                                   "machine to have at least <b>%1</b> of video memory.</p><p>Press "
                                                   "<b>Ignore</b> to switch to full-screen mode anyway or press <b>Cancel</b> to "
                                                   "cancel the operation.</p>").arg(UITranslator::formatSize(uMinVRAM)),
        QStringList() << QString() /* cancel button text */
                      << QApplication::translate("UIMessageCenter", "Ignore") /* ok button text */);
}

/* static */
bool UINotificationQuestion::confirmSwitchingScreenInFullscreen(quint64 uMinVRAM)
{
    return createBlockingQuestion(
        QApplication::translate("UIMessageCenter", "Still switch to another screen?"),
        QApplication::translate("UIMessageCenter", "<p>Could not change the guest screen to this host screen due to insufficient "
                                                   "guest video memory.</p><p>You should configure the virtual machine to have "
                                                   "at least <b>%1</b> of video memory.</p><p>Press <b>Ignore</b> to switch the "
                                                   "screen anyway or press <b>Cancel</b> to cancel the operation.</p>")
                                                   .arg(UITranslator::formatSize(uMinVRAM)),
        QStringList() << QString() /* cancel button text */
                      << QApplication::translate("UIMessageCenter", "Ignore") /* ok button text */);
}

UINotificationQuestion::UINotificationQuestion(const QString &strName,
                                               const QString &strDetails,
                                               const QStringList &buttonNames,
                                               bool fOkByDefault,
                                               const QString &strInternalName,
                                               const QString &strHelpKeyword)
    : UINotificationSimple(strName,
                           strDetails,
                           strInternalName,
                           strHelpKeyword)
    , m_buttonNames(buttonNames)
    , m_fOkByDefault(fOkByDefault)
    , m_enmResult(Question::Result_Cancel)
    , m_fDone(false)
{
}

UINotificationQuestion::~UINotificationQuestion()
{
    /* Remove questions from known: */
    m_questions.remove(internalName());
}

/* static */
void UINotificationQuestion::createQuestionInt(UINotificationCenter *pParent,
                                               const QString &strName,
                                               const QString &strDetails,
                                               const QStringList &buttonNames,
                                               bool fOkByDefault,
                                               const QString &strInternalName,
                                               const QString &strHelpKeyword)
{
    /* Make sure parent is set: */
    AssertPtr(pParent);
    UINotificationCenter *pEffectiveParent = pParent ? pParent : gpNotificationCenter;

    /* Check if question suppressed: */
    if (isSuppressed(strInternalName))
        return;

    /* Check if question already exists: */
    if (   !strInternalName.isEmpty()
        && m_questions.contains(strInternalName))
        return;

    /* Create question finally: */
    const QUuid uId = pEffectiveParent->append(new UINotificationQuestion(strName,
                                                                          strDetails,
                                                                          buttonNames,
                                                                          fOkByDefault,
                                                                          strInternalName,
                                                                          strHelpKeyword));
    if (!strInternalName.isEmpty())
        m_questions[strInternalName] = uId;
}

/* static */
int UINotificationQuestion::createBlockingQuestionInt(UINotificationCenter *pParent,
                                                      const QString &strName,
                                                      const QString &strDetails,
                                                      const QStringList &buttonNames,
                                                      bool fOkByDefault,
                                                      const QString &strInternalName,
                                                      const QString &strHelpKeyword)
{
    /* Make sure parent is set: */
    AssertPtr(pParent);
    UINotificationCenter *pEffectiveParent = pParent ? pParent : gpNotificationCenter;

    /* Check if question suppressed: */
    if (isSuppressed(strInternalName))
        return Question::Result_Accept;

    /* Create question finally: */
    QPointer<UINotificationQuestion> pQuestion = new UINotificationQuestion(strName,
                                                                            strDetails,
                                                                            buttonNames,
                                                                            fOkByDefault,
                                                                            strInternalName,
                                                                            strHelpKeyword);
    const int iResult = pEffectiveParent->showBlocking(pQuestion);
    delete pQuestion;
    return iResult;
}

/* static */
void UINotificationQuestion::createQuestion(const QString &strName,
                                            const QString &strDetails,
                                            const QStringList &buttonNames /* = QStringList() */,
                                            bool fOkByDefault /* = true */,
                                            const QString &strInternalName /* = QString() */,
                                            const QString &strHelpKeyword /* = QString() */,
                                            QWidget *pParent /* = 0 */)
{
    /* Acquire notification-center, make sure it's present: */
    UINotificationCenter *pCenter = UINotificationCenter::acquire(pParent);
    AssertPtrReturnVoid(pCenter);

    /* Redirect to wrapper above: */
    return createQuestionInt(pCenter, strName, strDetails, buttonNames, fOkByDefault, strInternalName, strHelpKeyword);
}

/* static */
int UINotificationQuestion::createBlockingQuestion(const QString &strName,
                                                   const QString &strDetails,
                                                   const QStringList &buttonNames /* = QStringList() */,
                                                   bool fOkByDefault /* = true */,
                                                   const QString &strInternalName /* = QString() */,
                                                   const QString &strHelpKeyword /* = QString() */,
                                                   QWidget *pParent /* = 0 */)
{
    /* Acquire notification-center, make sure it's present: */
    UINotificationCenter *pCenter = UINotificationCenter::acquire(pParent);
    AssertPtrReturn(pCenter, 0);

    /* Redirect to wrapper above: */
    return createBlockingQuestionInt(pCenter, strName, strDetails, buttonNames, fOkByDefault, strInternalName, strHelpKeyword);
}

/* static */
void UINotificationQuestion::destroyQuestion(const QString &strInternalName,
                                             UINotificationCenter *pParent /* = 0 */)
{
    /* Check if question really exists: */
    if (!m_questions.contains(strInternalName))
        return;

    /* Choose effective parent: */
    UINotificationCenter *pEffectiveParent = pParent ? pParent : gpNotificationCenter;

    /* Destroy question finally: */
    pEffectiveParent->revoke(m_questions.value(strInternalName));
    m_questions.remove(strInternalName);
}
