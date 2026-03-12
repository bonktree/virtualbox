/* $Id: UIMessageCenter.cpp 113378 2026-03-12 13:36:06Z sergey.dubov@oracle.com $ */
/** @file
 * VBox Qt GUI - UIMessageCenter class implementation.
 */

/*
 * Copyright (C) 2006-2026 Oracle and/or its affiliates.
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
#include <QThread>
#include <QWindow>

/* GUI includes: */
#include "QIMessageBox.h"
#include "UICommon.h"
#include "UIErrorString.h"
#include "UIExtraDataManager.h"
#include "UIGlobalSession.h"
#include "UIIconPool.h"
#include "UIMedium.h"
#include "UIMessageCenter.h"
#include "UIModalWindowManager.h"
#include "UIProgressDialog.h"
#include "UIVersion.h"
#include "VBoxAboutDlg.h"

/* COM includes: */
#include "CMachine.h"
#include "CSession.h"
#include "CVirtualBox.h"

/* Other VBox includes: */
#include <VBox/version.h>

/* static */
UIMessageCenter *UIMessageCenter::s_pInstance = 0;
UIMessageCenter *UIMessageCenter::instance() { return s_pInstance; }

/* static */
void UIMessageCenter::create()
{
    /* Make sure instance is NOT created yet: */
    if (s_pInstance)
    {
        AssertMsgFailed(("UIMessageCenter instance is already created!"));
        return;
    }

    /* Create instance: */
    new UIMessageCenter;
    /* Prepare instance: */
    s_pInstance->prepare();
}

/* static */
void UIMessageCenter::destroy()
{
    /* Make sure instance is NOT destroyed yet: */
    if (!s_pInstance)
    {
        AssertMsgFailed(("UIMessageCenter instance is already destroyed!"));
        return;
    }

    /* Cleanup instance: */
    s_pInstance->cleanup();
    /* Destroy instance: */
    delete s_pInstance;
}

void UIMessageCenter::setWarningShown(const QString &strWarningName, bool fWarningShown) const
{
    if (fWarningShown && !m_warnings.contains(strWarningName))
        m_warnings.append(strWarningName);
    else if (!fWarningShown && m_warnings.contains(strWarningName))
        m_warnings.removeAll(strWarningName);
}

bool UIMessageCenter::warningShown(const QString &strWarningName) const
{
    return m_warnings.contains(strWarningName);
}

int UIMessageCenter::message(QWidget *pParent, MessageType enmType,
                             const QString &strMessage,
                             const QString &strDetails,
                             const char *pcszAutoConfirmId /* = 0*/,
                             int iButton1 /* = 0*/,
                             int iButton2 /* = 0*/,
                             int iButton3 /* = 0*/,
                             const QString &strButtonText1 /* = QString() */,
                             const QString &strButtonText2 /* = QString() */,
                             const QString &strButtonText3 /* = QString() */,
                             const QString &strHelpKeyword /* = QString() */) const
{
    /* If this is NOT a GUI thread: */
    if (thread() != QThread::currentThread())
    {
        /* We have to throw a blocking signal
         * to show a message-box in the GUI thread: */
        emit sigToShowMessageBox(pParent, enmType,
                                 strMessage, strDetails,
                                 iButton1, iButton2, iButton3,
                                 strButtonText1, strButtonText2, strButtonText3,
                                 QString(pcszAutoConfirmId), strHelpKeyword);
        // Inter-thread communications are not yet implemented, so
        // we are not returning effective value, but zero otherwise.
        return 0;
    }

    /* In usual case we can chow a message-box directly: */
    return showMessageBox(pParent, enmType,
                          strMessage, strDetails,
                          iButton1, iButton2, iButton3,
                          strButtonText1, strButtonText2, strButtonText3,
                          QString(pcszAutoConfirmId), strHelpKeyword);
}

void UIMessageCenter::error(QWidget *pParent, MessageType enmType,
                           const QString &strMessage,
                           const QString &strDetails,
                           const char *pcszAutoConfirmId /* = 0*/,
                           const QString &strHelpKeyword /* = QString() */) const
{
    message(pParent, enmType, strMessage, strDetails, pcszAutoConfirmId,
            AlertButton_Ok | AlertButtonOption_Default | AlertButtonOption_Escape, 0 /* Button 2 */, 0 /* Button 3 */,
            QString() /* strButtonText1 */, QString() /* strButtonText2 */, QString() /* strButtonText3 */, strHelpKeyword);
}

bool UIMessageCenter::errorWithQuestion(QWidget *pParent, MessageType enmType,
                                        const QString &strMessage,
                                        const QString &strDetails,
                                        const char *pcszAutoConfirmId /* = 0*/,
                                        const QString &strOkButtonText /* = QString()*/,
                                        const QString &strCancelButtonText /* = QString()*/,
                                        const QString &strHelpKeyword /* = QString()*/) const
{
    return (message(pParent, enmType, strMessage, strDetails, pcszAutoConfirmId,
                    AlertButton_Ok | AlertButtonOption_Default,
                    AlertButton_Cancel | AlertButtonOption_Escape,
                    0 /* third button */,
                    strOkButtonText,
                    strCancelButtonText,
                    QString() /* third button text*/,
                    strHelpKeyword) &
            AlertButtonMask) == AlertButton_Ok;
}

void UIMessageCenter::alert(QWidget *pParent, MessageType enmType,
                           const QString &strMessage,
                           const char *pcszAutoConfirmId /* = 0*/,
                           const QString &strHelpKeyword /* = QString()*/) const
{
    error(pParent, enmType, strMessage, QString(), pcszAutoConfirmId, strHelpKeyword);
}

int UIMessageCenter::question(QWidget *pParent, MessageType enmType,
                              const QString &strMessage,
                              const char *pcszAutoConfirmId/* = 0*/,
                              int iButton1 /* = 0*/,
                              int iButton2 /* = 0*/,
                              int iButton3 /* = 0*/,
                              const QString &strButtonText1 /* = QString()*/,
                              const QString &strButtonText2 /* = QString()*/,
                              const QString &strButtonText3 /* = QString()*/) const
{
    return message(pParent, enmType, strMessage, QString(), pcszAutoConfirmId,
                   iButton1, iButton2, iButton3, strButtonText1, strButtonText2, strButtonText3);
}

bool UIMessageCenter::questionBinary(QWidget *pParent, MessageType enmType,
                                     const QString &strMessage,
                                     const char *pcszAutoConfirmId /* = 0*/,
                                     const QString &strOkButtonText /* = QString()*/,
                                     const QString &strCancelButtonText /* = QString()*/,
                                     bool fDefaultFocusForOk /* = true*/) const
{
    return fDefaultFocusForOk ?
           ((question(pParent, enmType, strMessage, pcszAutoConfirmId,
                      AlertButton_Ok | AlertButtonOption_Default,
                      AlertButton_Cancel | AlertButtonOption_Escape,
                      0 /* third button */,
                      strOkButtonText,
                      strCancelButtonText,
                      QString() /* third button */) &
             AlertButtonMask) == AlertButton_Ok) :
           ((question(pParent, enmType, strMessage, pcszAutoConfirmId,
                      AlertButton_Ok,
                      AlertButton_Cancel | AlertButtonOption_Default | AlertButtonOption_Escape,
                      0 /* third button */,
                      strOkButtonText,
                      strCancelButtonText,
                      QString() /* third button */) &
             AlertButtonMask) == AlertButton_Ok);
}

bool UIMessageCenter::showModalProgressDialog(CProgress &progress,
                                              const QString &strTitle,
                                              const QString &strImage /* = "" */,
                                              QWidget *pParent /* = 0*/,
                                              int cMinDuration /* = 2000 */)
{
    /* Prepare result: */
    bool fRc = false;

    /* Gather suitable dialog parent: */
    QWidget *pDlgParent = windowManager().realParentWindow(pParent ? pParent : windowManager().mainWindowShown());

    /* Prepare pixmap: */
    QPixmap pixmap;
    if (!strImage.isEmpty())
    {
        const qreal fDevicePixelRatio = pDlgParent && pDlgParent->windowHandle() ? pDlgParent->windowHandle()->devicePixelRatio() : 1;
        pixmap = UIIconPool::iconSet(strImage).pixmap(QSize(90, 90), fDevicePixelRatio);
    }

    /* Create progress-dialog: */
    QPointer<UIProgressDialog> pProgressDlg = new UIProgressDialog(progress, strTitle, &pixmap, cMinDuration, pDlgParent);
    if (pProgressDlg)
    {
        /* Register it as new parent: */
        windowManager().registerNewParent(pProgressDlg, pDlgParent);

        /* Run the dialog with the 350 ms refresh interval. */
        pProgressDlg->run(350);

        /* Make sure progress-dialog still valid: */
        if (pProgressDlg)
        {
            /* Delete progress-dialog: */
            delete pProgressDlg;
            fRc = true;
        }
    }

    /* Return result: */
    return fRc;
}

void UIMessageCenter::cannotFindLanguage(const QString &strLangId, const QString &strNlsPath) const
{
    alert(0, MessageType_Error,
          tr("<p>Could not find a language file for the language <b>%1</b> in the directory <b><nobr>%2</nobr></b>.</p>"
             "<p>The language will be temporarily reset to the system default language. "
             "Please go to the <b>Preferences</b> window which you can open from the <b>File</b> menu of the "
             "VirtualBox Manager window, and select one of the existing languages on the <b>Language</b> page.</p>")
             .arg(strLangId).arg(strNlsPath));
}

void UIMessageCenter::cannotLoadLanguage(const QString &strLangFile) const
{
    alert(0, MessageType_Error,
          tr("<p>Could not load the language file <b><nobr>%1</nobr></b>. "
             "<p>The language will be temporarily reset to English (built-in). "
             "Please go to the <b>Preferences</b> window which you can open from the <b>File</b> menu of the "
             "VirtualBox Manager window, and select one of the existing languages on the <b>Language</b> page.</p>")
             .arg(strLangFile));
}

void UIMessageCenter::cannotInitUserHome(const QString &strUserHome) const
{
    error(0, MessageType_Critical,
          tr("<p>Failed to initialize COM because the VirtualBox global "
             "configuration directory <b><nobr>%1</nobr></b> is not accessible. "
             "Please check the permissions of this directory and of its parent directory.</p>"
             "<p>The application will now terminate.</p>")
             .arg(strUserHome),
          UIErrorString::formatErrorInfo(COMErrorInfo()));
}

void UIMessageCenter::cannotInitCOM(HRESULT rc) const
{
    error(0, MessageType_Critical,
          tr("<p>Failed to initialize COM or to find the VirtualBox COM server. "
             "Most likely, the VirtualBox server is not running or failed to start.</p>"
             "<p>The application will now terminate.</p>"),
          UIErrorString::formatErrorInfo(COMErrorInfo(), rc));
}

void UIMessageCenter::cannotHandleRuntimeOption(const QString &strOption) const
{
    alert(0, MessageType_Error,
          tr("<b>%1</b> is an option for the VirtualBox VM runner (VirtualBoxVM) application, not the VirtualBox Manager.")
             .arg(strOption));
}

void UIMessageCenter::cannotStartSelector() const
{
    alert(0, MessageType_Critical,
          tr("<p>Cannot start the VirtualBox Manager due to local restrictions.</p>"
             "<p>The application will now terminate.</p>"));
}

void UIMessageCenter::cannotStartRuntime() const
{
    /* Prepare error string: */
    const QString strError = tr("<p>You must specify a machine to start, using the command line.</p><p>%1</p>",
                                "There will be a usage text passed as argument.");

    /* Prepare Usage, it can change in future: */
    const QString strTable = QString("<table cellspacing=0 style='white-space:pre'>%1</table>");
    const QString strUsage = tr("<tr>"
                                "<td>Usage: VirtualBoxVM --startvm &lt;name|UUID&gt;</td>"
                                "</tr>"
                                "<tr>"
                                "<td>Starts the VirtualBox virtual machine with the given "
                                "name or unique identifier (UUID).</td>"
                                "</tr>");

    /* Show error: */
    alert(0, MessageType_Error, strError.arg(strTable.arg(strUsage)));
}

bool UIMessageCenter::cannotRestoreSnapshot(const CMachine &machine, const QString &strSnapshotName, const QString &strMachineName) const
{
    error(0, MessageType_Error,
          tr("Failed to restore the snapshot <b>%1</b> of the virtual machine <b>%2</b>.")
             .arg(strSnapshotName, strMachineName),
          UIErrorString::formatErrorInfo(machine));
    return false;
}

bool UIMessageCenter::cannotRestoreSnapshot(const CProgress &progress, const QString &strSnapshotName, const QString &strMachineName) const
{
    error(0, MessageType_Error,
          tr("Failed to restore the snapshot <b>%1</b> of the virtual machine <b>%2</b>.")
             .arg(strSnapshotName, strMachineName),
          UIErrorString::formatErrorInfo(progress));
    return false;
}

void UIMessageCenter::cannotCreateVirtualBoxClient(const CVirtualBoxClient &comClient) const
{
    error(0, MessageType_Critical,
          tr("<p>Failed to create the VirtualBoxClient COM object.</p>"
             "<p>The application will now terminate.</p>"),
          UIErrorString::formatErrorInfo(comClient));
}

void UIMessageCenter::cannotAcquireVirtualBox(const CVirtualBoxClient &comClient) const
{
    QString err = tr("<p>Failed to acquire the VirtualBox COM object.</p>"
                     "<p>The application will now terminate.</p>");
#if defined(VBOX_WS_NIX) || defined(VBOX_WS_MAC)
    if (comClient.lastRC() == NS_ERROR_SOCKET_FAIL)
        err += tr("<p>The reason for this error are most likely wrong permissions of the IPC "
                  "daemon socket due to an installation problem. Please check the permissions of "
                  "<font color=blue>'/tmp'</font> and <font color=blue>'/tmp/.vbox-*-ipc/'</font></p>");
#endif
    error(0, MessageType_Critical, err, UIErrorString::formatErrorInfo(comClient));
}

void UIMessageCenter::cannotFindMachineByName(const CVirtualBox &comVBox, const QString &strName) const
{
    error(0, MessageType_Error,
          tr("There is no virtual machine named <b>%1</b>.")
             .arg(strName),
          UIErrorString::formatErrorInfo(comVBox));
}

void UIMessageCenter::cannotFindMachineById(const CVirtualBox &comVBox, const QUuid &uId) const
{
    error(0, MessageType_Error,
          tr("There is no virtual machine with the identifier <b>%1</b>.")
             .arg(uId.toString()),
          UIErrorString::formatErrorInfo(comVBox));
}

void UIMessageCenter::cannotSetExtraData(const CVirtualBox &comVBox, const QString &strKey, const QString &strValue)
{
    error(0, MessageType_Error,
          tr("Failed to set the global VirtualBox extra data for key <i>%1</i> to value <i>{%2}</i>.")
             .arg(strKey, strValue),
          UIErrorString::formatErrorInfo(comVBox));
}

void UIMessageCenter::cannotOpenSession(const CSession &comSession) const
{
    error(0, MessageType_Error,
          tr("Failed to create a new session."),
          UIErrorString::formatErrorInfo(comSession));
}

void UIMessageCenter::cannotOpenSession(const CMachine &comMachine) const
{
    error(0, MessageType_Error,
          tr("Failed to open a session for the virtual machine <b>%1</b>.")
             .arg(CMachine(comMachine).GetName()),
          UIErrorString::formatErrorInfo(comMachine));
}

void UIMessageCenter::cannotOpenSession(const CProgress &comProgress, const QString &strMachineName) const
{
    error(0, MessageType_Error,
          tr("Failed to open a session for the virtual machine <b>%1</b>.")
             .arg(strMachineName),
          UIErrorString::formatErrorInfo(comProgress));
}

void UIMessageCenter::cannotSetExtraData(const CMachine &machine, const QString &strKey, const QString &strValue)
{
    error(0, MessageType_Error,
          tr("Failed to set the extra data for key <i>%1</i> of machine <i>%2</i> to value <i>{%3}</i>.")
             .arg(strKey, CMachine(machine).GetName(), strValue),
          UIErrorString::formatErrorInfo(machine));
}

bool UIMessageCenter::cannotRemountMedium(const CMachine &machine, const UIMedium &medium, bool fMount,
                                          bool fRetry, QWidget *pParent /* = 0*/) const
{
    /* Compose the message: */
    QString strMessage;
    switch (medium.type())
    {
        case UIMediumDeviceType_DVD:
        {
            if (fMount)
            {
                strMessage = tr("<p>Unable to insert the virtual optical disk <nobr><b>%1</b></nobr> into the machine <b>%2</b>.</p>");
                if (fRetry)
                    strMessage += tr("<p>Would you like to try to force insertion of this disk?</p>");
            }
            else
            {
                strMessage = tr("<p>Unable to eject the virtual optical disk <nobr><b>%1</b></nobr> from the machine <b>%2</b>.</p>");
                if (fRetry)
                    strMessage += tr("<p>Would you like to try to force ejection of this disk?</p>");
            }
            break;
        }
        case UIMediumDeviceType_Floppy:
        {
            if (fMount)
            {
                strMessage = tr("<p>Unable to insert the virtual floppy disk <nobr><b>%1</b></nobr> into the machine <b>%2</b>.</p>");
                if (fRetry)
                    strMessage += tr("<p>Would you like to try to force insertion of this disk?</p>");
            }
            else
            {
                strMessage = tr("<p>Unable to eject the virtual floppy disk <nobr><b>%1</b></nobr> from the machine <b>%2</b>.</p>");
                if (fRetry)
                    strMessage += tr("<p>Would you like to try to force ejection of this disk?</p>");
            }
            break;
        }
        default:
            break;
    }
    /* Show the messsage: */
    if (fRetry)
        return errorWithQuestion(pParent, MessageType_Question,
                                 strMessage.arg(medium.isHostDrive() ? medium.name() : medium.location(), CMachine(machine).GetName()),
                                 UIErrorString::formatErrorInfo(machine),
                                 0 /* Auto Confirm ID */,
                                 tr("Force Unmount"));
    error(pParent, MessageType_Error,
          strMessage.arg(medium.isHostDrive() ? medium.name() : medium.location(), CMachine(machine).GetName()),
          UIErrorString::formatErrorInfo(machine));
    return false;
}

bool UIMessageCenter::confirmSettingsDiscarding(QWidget *pParent /* = 0 */) const
{
    if (   warningShown("confirmSettingsDiscarding")
        || warningShown("confirmSettingsReloading"))
        return false;
    setWarningShown("confirmSettingsDiscarding", true);

    const bool fResult = questionBinary(pParent, MessageType_Question,
                                        tr("<p>The machine settings were changed.</p>"
                                           "<p>Would you like to discard the changed settings or to keep editing them?</p>"),
                                        0 /* auto-confirm id */,
                                        tr("Discard changes"), tr("Keep editing"));

    setWarningShown("confirmSettingsDiscarding", false);

    return fResult;
}

bool UIMessageCenter::confirmSettingsReloading(QWidget *pParent /* = 0 */) const
{
    if (   warningShown("confirmSettingsReloading")
        || warningShown("confirmSettingsDiscarding"))
        return false;
    setWarningShown("confirmSettingsReloading", true);

    const bool fResult = questionBinary(pParent, MessageType_Question,
                                        tr("<p>The machine settings were changed while you were editing them. "
                                           "You currently have unsaved setting changes.</p>"
                                           "<p>Would you like to reload the changed settings or to keep your own changes?</p>"),
                                        0 /* auto-confirm id */,
                                        tr("Reload settings"), tr("Keep changes"));

    setWarningShown("confirmSettingsReloading", false);

    return fResult;
}

bool UIMessageCenter::confirmVisoDiscard(QWidget *pParent /* = 0*/) const
{
    return questionBinary(pParent, MessageType_Question,
                          tr("<p>To open a VISO file you will have to discard the current content.</p>"
                             "<p>Are you sure you want to proceed?</p>"),
                          0 /* auto-confirm id */,
                          tr("Discard") /* ok button text */,
                          QString() /* cancel button text */,
                          false /* ok button by default? */);
}

bool UIMessageCenter::warnAboutGuruMeditation(const QString &strLogFolder)
{
    return questionBinary(0, MessageType_GuruMeditation,
                          tr("<p>A critical error has occurred while running the virtual "
                             "machine and the machine execution has been stopped.</p>"
                             ""
                             "<p>For help, please see the Community section on "
                             "<a href=https://www.virtualbox.org>https://www.virtualbox.org</a> "
                             "or your support contract. Please provide the contents of the "
                             "log file <tt>VBox.log</tt> and the image file <tt>VBox.png</tt>, "
                             "which you can find in the <nobr><b>%1</b></nobr> directory, "
                             "as well as a description of what you were doing when this error happened. "
                             ""
                             "Note that you can also access the above files by selecting <b>Show Log</b> "
                             "from the <b>Machine</b> menu of the main VirtualBox window.</p>"
                             ""
                             "<p>Press <b>OK</b> if you want to power off the machine "
                             "or press <b>Ignore</b> if you want to leave it as is for debugging. "
                             "Please note that debugging requires special knowledge and tools, "
                             "so it is recommended to press <b>OK</b> now.</p>")
                             .arg(strLogFolder),
                          0 /* auto-confirm id */,
                          QIMessageBox::tr("OK"),
                          tr("Ignore"));
}

void UIMessageCenter::sltShowHelpWebDialog()
{
    uiCommon().openURL("https://www.virtualbox.org");
}

void UIMessageCenter::sltShowBugTracker()
{
    uiCommon().openURL("https://github.com/VirtualBox/virtualbox/issues");
}

void UIMessageCenter::sltShowForums()
{
    uiCommon().openURL("https://forums.virtualbox.org/");
}

void UIMessageCenter::sltShowOracle()
{
    uiCommon().openURL("https://www.oracle.com/us/technologies/virtualization/virtualbox/overview/index.html");
}

void UIMessageCenter::sltShowOnlineDocumentation()
{
    QString strUrl = QString("https://docs.oracle.com/en/virtualization/virtualbox/%1.%2/user/index.html").arg(VBOX_VERSION_MAJOR).arg(VBOX_VERSION_MINOR);
    uiCommon().openURL(strUrl);
}

void UIMessageCenter::sltShowHelpAboutDialog()
{
    CVirtualBox vbox = gpGlobalSession->virtualBox();
    const QString strFullVersion = UIVersionInfo::brandingIsActive()
                                 ? QString("%1 r%2 - %3").arg(vbox.GetVersion())
                                                         .arg(vbox.GetRevision())
                                                         .arg(UIVersionInfo::brandingGetKey("Name"))
                                 : QString("%1 r%2").arg(vbox.GetVersion())
                                                    .arg(vbox.GetRevision());
    (new VBoxAboutDlg(windowManager().mainWindowShown(), strFullVersion))->show();
}

void UIMessageCenter::sltResetSuppressedMessages()
{
    /* Nullify suppressed message list: */
    gEDataManager->setSuppressedMessages(QStringList());
}

void UIMessageCenter::sltShowMessageBox(QWidget *pParent, MessageType enmType,
                                        const QString &strMessage, const QString &strDetails,
                                        int iButton1, int iButton2, int iButton3,
                                        const QString &strButtonText1, const QString &strButtonText2, const QString &strButtonText3,
                                        const QString &strAutoConfirmId, const QString &strHelpKeyword) const
{
    /* Now we can show a message-box directly: */
    showMessageBox(pParent, enmType,
                   strMessage, strDetails,
                   iButton1, iButton2, iButton3,
                   strButtonText1, strButtonText2, strButtonText3,
                   strAutoConfirmId, strHelpKeyword);
}

UIMessageCenter::UIMessageCenter()
{
    /* Assign instance: */
    s_pInstance = this;
}

UIMessageCenter::~UIMessageCenter()
{
    /* Unassign instance: */
    s_pInstance = 0;
}

void UIMessageCenter::prepare()
{
    /* Prepare interthread connection: */
    qRegisterMetaType<MessageType>();
    connect(this, &UIMessageCenter::sigToShowMessageBox,
            this, &UIMessageCenter::sltShowMessageBox,
            Qt::BlockingQueuedConnection);

    /* Translations for Main.
     * Please make sure they corresponds to the strings coming from Main one-by-one symbol! */
    tr("Could not load the Host USB Proxy Service (VERR_FILE_NOT_FOUND). The service might not be installed on the host computer");
    tr("VirtualBox is not currently allowed to access USB devices.  You can change this by adding your user to the 'vboxusers' group.  Please see the user guide for a more detailed explanation");
    tr("VirtualBox is not currently allowed to access USB devices.  You can change this by allowing your user to access the 'usbfs' folder and files.  Please see the user guide for a more detailed explanation");
    tr("The USB Proxy Service has not yet been ported to this host");
    tr("Could not load the Host USB Proxy service");
}

void UIMessageCenter::cleanup()
{
     /* Nothing for now... */
}

int UIMessageCenter::showMessageBox(QWidget *pParent, MessageType enmType,
                                    const QString &strMessage, const QString &strDetails,
                                    int iButton1, int iButton2, int iButton3,
                                    const QString &strButtonText1, const QString &strButtonText2, const QString &strButtonText3,
                                    const QString &strAutoConfirmId, const QString &strHelpKeyword) const
{
    /* Choose the 'default' button: */
    if (iButton1 == 0 && iButton2 == 0 && iButton3 == 0)
        iButton1 = AlertButton_Ok | AlertButtonOption_Default;

    /* Check if message-box was auto-confirmed before: */
    QStringList confirmedMessageList;
    if (!strAutoConfirmId.isEmpty())
    {
        const QUuid uID = uiCommon().uiType() == UIType_RuntimeUI
                        ? uiCommon().managedVMUuid()
                        : UIExtraDataManager::GlobalID;
        confirmedMessageList = gEDataManager->suppressedMessages(uID);
        if (   confirmedMessageList.contains(strAutoConfirmId)
            || confirmedMessageList.contains("allMessageBoxes")
            || confirmedMessageList.contains("all") )
        {
            int iResultCode = AlertOption_AutoConfirmed;
            if (iButton1 & AlertButtonOption_Default)
                iResultCode |= (iButton1 & AlertButtonMask);
            if (iButton2 & AlertButtonOption_Default)
                iResultCode |= (iButton2 & AlertButtonMask);
            if (iButton3 & AlertButtonOption_Default)
                iResultCode |= (iButton3 & AlertButtonMask);
            return iResultCode;
        }
    }

    /* Choose title and icon: */
    QString title;
    AlertIconType icon;
    switch (enmType)
    {
        default:
        case MessageType_Info:
            title = tr("VirtualBox - Information", "msg box title");
            icon = AlertIconType_Information;
            break;
        case MessageType_Question:
            title = tr("VirtualBox - Question", "msg box title");
            icon = AlertIconType_Question;
            break;
        case MessageType_Warning:
            title = tr("VirtualBox - Warning", "msg box title");
            icon = AlertIconType_Warning;
            break;
        case MessageType_Error:
            title = tr("VirtualBox - Error", "msg box title");
            icon = AlertIconType_Critical;
            break;
        case MessageType_Critical:
            title = tr("VirtualBox - Critical Error", "msg box title");
            icon = AlertIconType_Critical;
            break;
        case MessageType_GuruMeditation:
            title = "VirtualBox - Guru Meditation"; /* don't translate this */
            icon = AlertIconType_GuruMeditation;
            break;
    }

    /* Create message-box: */
    QWidget *pMessageBoxParent = windowManager().realParentWindow(pParent ? pParent : windowManager().mainWindowShown());
    const QString strHackValue = gEDataManager->extraDataString("GUI/Hack/MakeMessageBoxParentless");
    QPointer<QIMessageBox> pMessageBox = new QIMessageBox(title, strMessage, icon,
                                                          iButton1, iButton2, iButton3,
                                                          strHackValue == "true" ? 0 : pMessageBoxParent,
                                                          strHelpKeyword);
    windowManager().registerNewParent(pMessageBox, pMessageBoxParent);

    /* Prepare auto-confirmation check-box: */
    if (!strAutoConfirmId.isEmpty())
    {
        pMessageBox->setFlagText(tr("Do not show this message again", "msg box flag"));
        pMessageBox->setFlagChecked(false);
    }

    /* Configure details: */
    if (!strDetails.isEmpty())
        pMessageBox->setDetailsText(strDetails);

    /* Configure button-text: */
    if (!strButtonText1.isNull())
        pMessageBox->setButtonText(0, strButtonText1);
    if (!strButtonText2.isNull())
        pMessageBox->setButtonText(1, strButtonText2);
    if (!strButtonText3.isNull())
        pMessageBox->setButtonText(2, strButtonText3);

    /* Show message-box: */
    int iResultCode = pMessageBox->exec();

    /* Make sure message-box still valid: */
    if (!pMessageBox)
        return iResultCode;

    /* Remember auto-confirmation check-box value: */
    if (!strAutoConfirmId.isEmpty())
    {
        if (pMessageBox->flagChecked())
        {
            confirmedMessageList << strAutoConfirmId;
            gEDataManager->setSuppressedMessages(confirmedMessageList);
        }
    }

    /* Delete message-box: */
    delete pMessageBox;

    /* Return result-code: */
    return iResultCode;
}
