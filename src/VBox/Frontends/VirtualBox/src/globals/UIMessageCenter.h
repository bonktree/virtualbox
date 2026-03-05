/* $Id: UIMessageCenter.h 113265 2026-03-05 08:50:41Z sergey.dubov@oracle.com $ */
/** @file
 * VBox Qt GUI - UIMessageCenter class declaration.
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

#ifndef FEQT_INCLUDED_SRC_globals_UIMessageCenter_h
#define FEQT_INCLUDED_SRC_globals_UIMessageCenter_h
#ifndef RT_WITHOUT_PRAGMA_ONCE
# pragma once
#endif

/* Qt includes: */
#include <QObject>

/* GUI includes: */
#include "UILibraryDefs.h"
#include "UIMediumDefs.h"

/* COM includes: */
#include "CProgress.h"

/* Forward declarations: */
class UIMedium;
struct StorageSlot;
#ifdef VBOX_WITH_DRAG_AND_DROP
class CGuest;
#endif

/** Possible message types. */
enum MessageType
{
    MessageType_Info = 1,
    MessageType_Question,
    MessageType_Warning,
    MessageType_Error,
    MessageType_Critical,
    MessageType_GuruMeditation
};
Q_DECLARE_METATYPE(MessageType);

/** Singleton QObject extension
  * providing GUI with corresponding messages. */
class SHARED_LIBRARY_STUFF UIMessageCenter : public QObject
{
    Q_OBJECT;

signals:

    /** Asks to show message-box.
      * @param  pParent           Brings the message-box parent.
      * @param  enmType           Brings the message-box type.
      * @param  strMessage        Brings the message.
      * @param  strDetails        Brings the details.
      * @param  iButton1          Brings the button 1 type.
      * @param  iButton2          Brings the button 2 type.
      * @param  iButton3          Brings the button 3 type.
      * @param  strButtonText1    Brings the button 1 text.
      * @param  strButtonText2    Brings the button 2 text.
      * @param  strButtonText3    Brings the button 3 text.
      * @param  strAutoConfirmId  Brings whether this message can be auto-confirmed. */
    void sigToShowMessageBox(QWidget *pParent, MessageType enmType,
                             const QString &strMessage, const QString &strDetails,
                             int iButton1, int iButton2, int iButton3,
                             const QString &strButtonText1, const QString &strButtonText2, const QString &strButtonText3,
                             const QString &strAutoConfirmId, const QString &strHelpKeyword) const;

public:

    /** Creates message-center singleton. */
    static void create();
    /** Destroys message-center singleton. */
    static void destroy();

    /** Defines whether warning with particular @a strWarningName is @a fShown. */
    void setWarningShown(const QString &strWarningName, bool fShown) const;
    /** Returns whether warning with particular @a strWarningName is shown. */
    bool warningShown(const QString &strWarningName) const;

    /** Shows a general type of 'Message'.
      * @param  pParent            Brings the message-box parent.
      * @param  enmType            Brings the message-box type.
      * @param  strMessage         Brings the message.
      * @param  strDetails         Brings the details.
      * @param  pcszAutoConfirmId  Brings the auto-confirm ID.
      * @param  iButton1           Brings the button 1 type.
      * @param  iButton2           Brings the button 2 type.
      * @param  iButton3           Brings the button 3 type.
      * @param  strButtonText1     Brings the button 1 text.
      * @param  strButtonText2     Brings the button 2 text.
      * @param  strButtonText3     Brings the button 3 text.
      * @param  strHelpKeyword     Brings the help keyword string. */
    int message(QWidget *pParent, MessageType enmType,
                const QString &strMessage, const QString &strDetails,
                const char *pcszAutoConfirmId = 0,
                int iButton1 = 0, int iButton2 = 0, int iButton3 = 0,
                const QString &strButtonText1 = QString(),
                const QString &strButtonText2 = QString(),
                const QString &strButtonText3 = QString(),
                const QString &strHelpKeyword = QString()) const;

    /** Shows an 'Error' type of 'Message'.
      * Provides single Ok button.
      * @param  pParent            Brings the message-box parent.
      * @param  enmType            Brings the message-box type.
      * @param  strMessage         Brings the message.
      * @param  strDetails         Brings the details.
      * @param  pcszAutoConfirmId  Brings the auto-confirm ID.
      * @param  strHelpKeyword     Brings the help keyword string. */
    void error(QWidget *pParent, MessageType enmType,
               const QString &strMessage,
               const QString &strDetails,
               const char *pcszAutoConfirmId = 0,
               const QString &strHelpKeyword = QString()) const;

    /** Shows an 'Error with Question' type of 'Message'.
      * Provides Ok and Cancel buttons (called same way by default).
      * @param  pParent              Brings the message-box parent.
      * @param  enmType              Brings the message-box type.
      * @param  strMessage           Brings the message.
      * @param  strDetails           Brings the details.
      * @param  pcszAutoConfirmId    Brings the auto-confirm ID.
      * @param  strOkButtonText      Brings the Ok button text.
      * @param  strCancelButtonText  Brings the Cancel button text.
      * @param  strHelpKeyword     Brings the help keyword string. */
    bool errorWithQuestion(QWidget *pParent, MessageType enmType,
                           const QString &strMessage,
                           const QString &strDetails,
                           const char *pcszAutoConfirmId = 0,
                           const QString &strOkButtonText = QString(),
                           const QString &strCancelButtonText = QString(),
                           const QString &strHelpKeyword = QString()) const;

    /** Shows an 'Alert' type of 'Error'.
      * Omit details.
      * @param  pParent            Brings the message-box parent.
      * @param  enmType            Brings the message-box type.
      * @param  strMessage         Brings the message.
      * @param  pcszAutoConfirmId  Brings the auto-confirm ID.
      * @param  strHelpKeyword     Brings the help keyword string. */
    void alert(QWidget *pParent, MessageType enmType,
               const QString &strMessage,
               const char *pcszAutoConfirmId = 0,
               const QString &strHelpKeyword = QString()) const;

    /** Shows a 'Question' type of 'Message'.
      * Omit details.
      * @param  pParent            Brings the message-box parent.
      * @param  enmType            Brings the message-box type.
      * @param  strMessage         Brings the message.
      * @param  pcszAutoConfirmId  Brings the auto-confirm ID.
      * @param  iButton1           Brings the button 1 type.
      * @param  iButton2           Brings the button 2 type.
      * @param  iButton3           Brings the button 3 type.
      * @param  strButtonText1     Brings the button 1 text.
      * @param  strButtonText2     Brings the button 2 text.
      * @param  strButtonText3     Brings the button 3 text. */
    int question(QWidget *pParent, MessageType enmType,
                 const QString &strMessage,
                 const char *pcszAutoConfirmId = 0,
                 int iButton1 = 0, int iButton2 = 0, int iButton3 = 0,
                 const QString &strButtonText1 = QString(),
                 const QString &strButtonText2 = QString(),
                 const QString &strButtonText3 = QString()) const;

    /** Shows a 'Binary' type of 'Question'.
      * Omit details. Provides Ok and Cancel buttons (called same way by default).
      * @param  pParent              Brings the message-box parent.
      * @param  enmType              Brings the message-box type.
      * @param  strMessage           Brings the message.
      * @param  pcszAutoConfirmId    Brings the auto-confirm ID.
      * @param  strOkButtonText      Brings the button 1 text.
      * @param  strCancelButtonText  Brings the button 2 text.
      * @param  fDefaultFocusForOk   Brings whether Ok button should be focused initially. */
    bool questionBinary(QWidget *pParent, MessageType enmType,
                        const QString &strMessage,
                        const char *pcszAutoConfirmId = 0,
                        const QString &strOkButtonText = QString(),
                        const QString &strCancelButtonText = QString(),
                        bool fDefaultFocusForOk = true) const;

    /** Shows a 'Trinary' type of 'Question'.
      * Omit details. Provides Yes, No and Cancel buttons (called same way by default).
      * @param  pParent               Brings the message-box parent.
      * @param  enmType               Brings the message-box type.
      * @param  strMessage            Brings the message.
      * @param  pcszAutoConfirmId     Brings the auto-confirm ID.
      * @param  strChoice1ButtonText  Brings the button 1 text.
      * @param  strChoice2ButtonText  Brings the button 2 text.
      * @param  strCancelButtonText   Brings the button 3 text. */
    int questionTrinary(QWidget *pParent, MessageType enmType,
                        const QString &strMessage,
                        const char *pcszAutoConfirmId = 0,
                        const QString &strChoice1ButtonText = QString(),
                        const QString &strChoice2ButtonText = QString(),
                        const QString &strCancelButtonText = QString()) const;

    /** Shows a general type of 'Message with Option'.
      * @param  pParent              Brings the message-box parent.
      * @param  enmType              Brings the message-box type.
      * @param  strMessage           Brings the message.
      * @param  strOptionText        Brings the option text.
      * @param  fDefaultOptionValue  Brings the default option value.
      * @param  iButton1             Brings the button 1 type.
      * @param  iButton2             Brings the button 2 type.
      * @param  iButton3             Brings the button 3 type.
      * @param  strButtonText1       Brings the button 1 text.
      * @param  strButtonText2       Brings the button 2 text.
      * @param  strButtonText3       Brings the button 3 text. */
    int messageWithOption(QWidget *pParent, MessageType enmType,
                          const QString &strMessage,
                          const QString &strOptionText,
                          bool fDefaultOptionValue = true,
                          int iButton1 = 0, int iButton2 = 0, int iButton3 = 0,
                          const QString &strButtonText1 = QString(),
                          const QString &strButtonText2 = QString(),
                          const QString &strButtonText3 = QString()) const;

    /** Shows modal progress-dialog.
      * @param  comProgress   Brings the progress this dialog is based on.
      * @param  strTitle      Brings the title.
      * @param  strImage      Brings the image.
      * @param  pParent       Brings the parent.
      * @param  cMinDuration  Brings the minimum diration to show this dialog after expiring it. */
    bool showModalProgressDialog(CProgress &comProgress, const QString &strTitle,
                                 const QString &strImage = "", QWidget *pParent = 0,
                                 int cMinDuration = 2000);

    /** @name Startup warnings.
      * @{ */
        void cannotFindLanguage(const QString &strLangId, const QString &strNlsPath) const;
        void cannotLoadLanguage(const QString &strLangFile) const;

        void cannotInitUserHome(const QString &strUserHome) const;
        void cannotInitCOM(HRESULT rc) const;

        void cannotHandleRuntimeOption(const QString &strOption) const;

        void cannotStartSelector() const;
        void cannotStartRuntime() const;

        bool cannotRestoreSnapshot(const CMachine &machine, const QString &strSnapshotName, const QString &strMachineName) const;
        bool cannotRestoreSnapshot(const CProgress &progress, const QString &strSnapshotName, const QString &strMachineName) const;
    /** @} */

    /** @name General COM warnings.
      * @{ */
        void cannotCreateVirtualBoxClient(const CVirtualBoxClient &comClient) const;
        void cannotAcquireVirtualBox(const CVirtualBoxClient &comClient) const;

        void cannotFindMachineByName(const CVirtualBox &comVBox, const QString &strName) const;
        void cannotFindMachineById(const CVirtualBox &comVBox, const QUuid &uId) const;
        void cannotSetExtraData(const CVirtualBox &comVBox, const QString &strKey, const QString &strValue);

        void cannotOpenSession(const CSession &comSession) const;
        void cannotOpenSession(const CMachine &comMachine) const;
        void cannotOpenSession(const CProgress &comProgress, const QString &strMachineName) const;

        void cannotSetExtraData(const CMachine &machine, const QString &strKey, const QString &strValue);
        bool cannotRemountMedium(const CMachine &machine, const UIMedium &medium,
                                 bool fMount, bool fRetry, QWidget *pParent = 0) const;
    /** @} */

    /** @name Common warnings.
      * @{ */
        bool confirmSettingsDiscarding(QWidget *pParent = 0) const;
        bool confirmSettingsReloading(QWidget *pParent = 0) const;
    /** @} */

    /** @name VirtualBox Manager / Chooser Pane warnings.
      * @{ */
        int confirmMachineRemoval(const QList<CMachine> &machines) const;
        int confirmCloudMachineRemoval(const QList<CCloudMachine> &machines) const;
    /** @} */

    /** @name VirtualBox Manager / Snapshot Pane warnings.
      * @{ */
        int confirmSnapshotRestoring(const QString &strSnapshotName, bool fAlsoCreateNewSnapshot) const;
    /** @} */

    /** @name VirtualBox Manager / Media Manager warnings.
      * @{ */
        bool confirmVisoDiscard(QWidget *pParent = 0) const;
        int  confirmUnattendedFilesRemoval(QWidget *pParent = 0) const;
    /** @} */

    /** @name Runtime UI warnings.
      * @{ */
        bool warnAboutGuruMeditation(const QString &strLogFolder);
    /** @} */

    /** @name VirtualBox Manager / Wizard warnings.
      * @{ */
        /// @todo move to notification-center after wizards get theirs.. :)
        bool confirmHardDisklessMachine(QWidget *pParent = 0) const;
        bool confirmExportMachinesInSaveState(const QStringList &machineNames, QWidget *pParent = 0) const;
        bool confirmOverridingFile(const QString &strPath, QWidget *pParent = 0) const;
        bool confirmOverridingFiles(const QVector<QString> &strPaths, QWidget *pParent = 0) const;
    /** @} */

    /** @name VirtualBox Manager / FD Creation Dialog warnings.
      * @{ */
        void cannotCreateMediumStorage(const CVirtualBox &comVBox, const QString &strLocation, QWidget *pParent = 0) const;
    /** @} */

public slots:

    /* Handlers: Help menu stuff: */
    void sltShowHelpWebDialog();
    void sltShowBugTracker();
    void sltShowForums();
    void sltShowOracle();
    void sltShowOnlineDocumentation();
    void sltShowHelpAboutDialog();
    void sltResetSuppressedMessages();

private slots:

    /** Shows message-box.
      * @param  pParent           Brings the message-box parent.
      * @param  enmType           Brings the message-box type.
      * @param  strMessage        Brings the message.
      * @param  strDetails        Brings the details.
      * @param  iButton1          Brings the button 1 type.
      * @param  iButton2          Brings the button 2 type.
      * @param  iButton3          Brings the button 3 type.
      * @param  strButtonText1    Brings the button 1 text.
      * @param  strButtonText2    Brings the button 2 text.
      * @param  strButtonText3    Brings the button 3 text.
      * @param  strAutoConfirmId  Brings whether this message can be auto-confirmed.
      * @param  strHelpKeyword    Brings the help keyword string. */
    void sltShowMessageBox(QWidget *pParent, MessageType enmType,
                           const QString &strMessage, const QString &strDetails,
                           int iButton1, int iButton2, int iButton3,
                           const QString &strButtonText1, const QString &strButtonText2, const QString &strButtonText3,
                           const QString &strAutoConfirmId, const QString &strHelpKeyword) const;

private:

    /** Constructs message-center. */
    UIMessageCenter();
    /** Destructs message-center. */
    ~UIMessageCenter();

    /** Prepares all. */
    void prepare();
    /** Cleanups all. */
    void cleanup();

    /** Shows message-box.
      * @param  pParent           Brings the message-box parent.
      * @param  enmType           Brings the message-box type.
      * @param  strMessage        Brings the message.
      * @param  strDetails        Brings the details.
      * @param  iButton1          Brings the button 1 type.
      * @param  iButton2          Brings the button 2 type.
      * @param  iButton3          Brings the button 3 type.
      * @param  strButtonText1    Brings the button 1 text.
      * @param  strButtonText2    Brings the button 2 text.
      * @param  strButtonText3    Brings the button 3 text.
      * @param  strAutoConfirmId  Brings whether this message can be auto-confirmed.
      * @param  strHelpKeyword    Brings the help keyowrd. */
    int showMessageBox(QWidget *pParent, MessageType type,
                       const QString &strMessage, const QString &strDetails,
                       int iButton1, int iButton2, int iButton3,
                       const QString &strButtonText1, const QString &strButtonText2, const QString &strButtonText3,
                       const QString &strAutoConfirmId, const QString &strHelpKeyword) const;

    /** Holds the list of shown warnings. */
    mutable QStringList m_warnings;

    /** Holds the singleton message-center instance. */
    static UIMessageCenter *s_pInstance;
    /** Returns the singleton message-center instance. */
    static UIMessageCenter *instance();
    /** Allows for shortcut access. */
    friend UIMessageCenter &msgCenter();
};

/** Singleton Message Center 'official' name. */
inline UIMessageCenter &msgCenter() { return *UIMessageCenter::instance(); }

#endif /* !FEQT_INCLUDED_SRC_globals_UIMessageCenter_h */
