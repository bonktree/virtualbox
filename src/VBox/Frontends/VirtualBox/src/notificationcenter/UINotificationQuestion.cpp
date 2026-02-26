/* $Id: UINotificationQuestion.cpp 113175 2026-02-26 12:33:20Z sergey.dubov@oracle.com $ */
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
#include "UINotificationCenter.h"
#include "UINotificationQuestion.h"


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
        QStringList() << QApplication::translate("UIMessageCenter", "Ignore") /* cancel button text */
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
                      << QApplication::translate("UIMessageCenter", "Rename") /* ok button text */);
}

/* static */
bool UINotificationQuestion::confirmMachineItemRemoval(const QString &strNames)
{
    return createBlockingQuestion(
        QApplication::translate("UIMessageCenter", "Remove machine items?"),
        QApplication::translate("UIMessageCenter", "<p>Remove these virtual machine items from the machine "
                                                   "list?</p><p><b>%1</b></p>").arg(strNames),
        QStringList() << QString() /* cancel button text */
                      << QApplication::translate("UIMessageCenter", "Remove") /* ok button text */,
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
                      << QApplication::translate("UIMessageCenter", "Delete") /* ok button text */);
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
                      << QApplication::translate("UIMessageCenter", "Remove", "medium") /* ok button text */,
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
