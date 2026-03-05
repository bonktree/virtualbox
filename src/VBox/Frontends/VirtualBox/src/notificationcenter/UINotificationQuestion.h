/* $Id: UINotificationQuestion.h 113269 2026-03-05 13:44:15Z sergey.dubov@oracle.com $ */
/** @file
 * VBox Qt GUI - Various UINotificationQuestion declarations.
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

#ifndef FEQT_INCLUDED_SRC_notificationcenter_UINotificationQuestion_h
#define FEQT_INCLUDED_SRC_notificationcenter_UINotificationQuestion_h
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
class UIMedium;
class UINotificationCenter;

/** Question related stuff. */
namespace Question
{
    /** Result options. */
    enum Result
    {
        Result_Cancel = 0,
        Result_Accept,
        Result_AcceptAlternative
    };
}

/** UINotificationSimple extension for question functionality. */
class SHARED_LIBRARY_STUFF UINotificationQuestion : public UINotificationSimple
{
    Q_OBJECT;

public:

    /** Returns the button names. */
    QStringList buttonNames() const { return m_buttonNames; }
    /** Returns whether Ok button should be default one. */
    bool isOkByDefault() const { return m_fOkByDefault; }

    /** Returns the result. */
    Question::Result result() const { return m_enmResult; }
    /** Defines the @a enmResult. */
    void setResult(Question::Result enmResult) { m_enmResult = enmResult; m_fDone = true; }

    /** Returns whether object is done. */
    virtual bool isDone() const RT_OVERRIDE RT_FINAL { return m_fDone; }

    /** @name General VirtualBox Manager warnings.
      * @{ */
        /** Confirm checking inaccessible media. */
        static bool confirmCheckingInaccessibleMedia();
    /** @} */

    /** @name VirtualBox Manager warnings / Modify VM
      * @{ */
        /** Confirms creation of the @a strPath for the machine to move in. */
        static bool confirmCreatingPath(const QString &strPath);

        /** Confirms automatic @a strName collision resolve inside group with @a strGroupName. */
        static bool confirmAutomaticCollisionResolve(const QString &strName, const QString &strGroupName);

        /** Confirms machine item removal for @a strNames specified. */
        static bool confirmMachineItemRemoval(const QString &strNames);

        /** Confirms removal for the snapshot with @a strName specified. */
        static bool confirmSnapshotRemoval(const QString &strName);
    /** @} */

    /** @name VirtualBox Manager warnings / Control VM
      * @{ */
        /** Confirms starting machines with @a strNames specified. */
        static bool confirmStartMultipleMachines(const QString &strNames);
        /** Confirms reset for the machine with @a strNames specified. */
        static bool confirmResetMachine(const QString &strNames);
        /** Confirms sending ACPI shutdown signal for machines with @a strNames specified. */
        static bool confirmACPIShutdownMachine(const QString &strNames);
        /** Confirms powering off machines with @a strNames specified. */
        static bool confirmPowerOffMachine(const QString &strNames);
        /** Confirms discarding saved state for machines with @a strNames specified. */
        static bool confirmDiscardSavedState(const QString &strNames);

        /** Confirms terminating cloud instance for machines with @a strNames specified. */
        static bool confirmTerminateCloudInstance(const QString &strNames);
    /** @} */

    /** @name Advanced Settings Dialog warnings
      * @{ */
        /** Confirms removal of the last DVD device. */
        static bool confirmRemovingOfLastDVDDevice(QWidget *pParent);
        /** Confirms storage bus change with optical devices removal. */
        static bool confirmStorageBusChangeWithOpticalRemoval(QWidget *pParent);
        /** Confirms storage bus change with excessive devices removal. */
        static bool confirmStorageBusChangeWithExcessiveRemoval(QWidget *pParent);
        /** Confirms canceling port forwarding dialog. */
        static bool confirmCancelingPortForwardingDialog(QWidget *pParent);
        /** Confirms restoring default keys. */
        static bool confirmRestoringDefaultKeys(QWidget *pParent);
    /** @} */

    /** @name Extension Pack Manager warnings
      * @{ */
        /** Confirms installing extension pack. */
        static bool confirmInstallExtensionPack(const QString &strPackName,
                                                const QString &strPackVersion,
                                                const QString &strPackDescription,
                                                QWidget *pParent);
        /** Confirms replacing extension pack. */
        static bool confirmReplaceExtensionPack(const QString &strPackName,
                                                const QString &strPackVersionNew,
                                                const QString &strPackVersionOld,
                                                const QString &strPackDescription,
                                                QWidget *pParent);
        /** Confirms removing extension pack. */
        static bool confirmRemoveExtensionPack(const QString &strPackName,
                                               QWidget *pParent);
    /** @} */

    /** @name VirtualBox Manager / Media Manager warnings.
      * @{ */
        /** Confirms medium releasing. */
        static bool confirmMediumRelease(const UIMedium &guiMedium,
                                         bool fInduced,
                                         const QStringList &usage,
                                         QWidget *pParent);

        /** Confirms medium removal. */
        static bool confirmMediumRemoval(const UIMedium &guiMedium,
                                         QWidget *pParent);
        /** Confirms hard disk storage destruction. */
        static int confirmDeleteHardDiskStorage(const QString &strLocation,
                                                QWidget *pParent);

        /** Confirms clearing inaccessible media. */
        static bool confirmInaccesibleMediaClear(const QStringList &media,
                                                 UIMediumDeviceType enmType,
                                                 QWidget *pParent);
    /** @} */

    /** @name Network Manager warnings.
      * @{ */
        /** Confirms cloud network removal. */
        static bool confirmCloudNetworkRemoval(const QString &strName,
                                               QWidget *pParent);
        /** Confirms host network interface removal. */
        static bool confirmHostNetworkInterfaceRemoval(const QString &strName,
                                                       QWidget *pParent);
        /** Confirms host-only network removal. */
        static bool confirmHostOnlyNetworkRemoval(const QString &strName,
                                                  QWidget *pParent);
        /** Confirms NAT network removal. */
        static bool confirmNATNetworkRemoval(const QString &strName,
                                             QWidget *pParent);
    /** @} */

    /** @name Cloud Profile Manager warnings.
      * @{ */
        /** Confirms cloud profile removal. */
        static bool confirmCloudProfileRemoval(const QString &strName, QWidget *pParent);
        /** Confirms cloud profiles import. */
        static bool confirmCloudProfilesImport(QWidget *pParent);
    /** @} */

    /** @name Cloud Console Manager warnings.
      * @{ */
        /** Confirms cloud console application removal. */
        static bool confirmCloudConsoleApplicationRemoval(const QString &strName);
        /** Confirms cloud console profile removal. */
        static bool confirmCloudConsoleProfileRemoval(const QString &strName);
    /** @} */

#ifdef VBOX_GUI_WITH_NETWORK_MANAGER
    /** @name Downloader warnings.
      * @{ */
        /** Confirms looking for guest additions. */
        static bool confirmLookingForGuestAdditions();
        /** Confirms downloading guest additions. */
        static bool confirmDownloadingGuestAdditions(const QString &strUrl, qulonglong uSize);
        /** Confirms mounting guest additions. */
        static bool confirmMountingGuestAdditions(const QString &strUrl, const QString &strSrc);

        /** Confirms looking for extension pack. */
        static bool confirmLookingForExtensionPack(const QString &strExtPackName, const QString &strExtPackVersion);
        /** Confirms downloading extension pack. */
        static bool confirmDownloadingExtensionPack(const QString &strExtPackName, const QString &strURL, qulonglong uSize);
        /** Confirms installing extension pack. */
        static bool confirmInstallingExtentionPack(const QString &strExtPackName, const QString &strFrom, const QString &strTo);
        /** Confirms deleting extension pack file. */
        static bool confirmDeletingExtentionPackFile(const QString &strTo);
        /** Confirms deleting old extension pack files. */
        static bool confirmDeletingOldExtentionPackFiles(const QStringList &strFiles);
    /** @} */
#endif /* VBOX_GUI_WITH_NETWORK_MANAGER */

    /** @name Wizard warnings.
      * @{ */
        /** Confirms overriding file. */
        static bool confirmOverridingFile(const QString &strPath, QWidget *pParent = 0);
        /** Confirms overriding files. */
        static bool confirmOverridingFiles(const QVector<QString> &strPaths, QWidget *pParent);
    /** @} */

    /** @name Runtime UI warnings.
      * @{ */
        /** Confirms network interface choice. */
        static bool warnAboutNetworkInterfaceNotFound(const QString &strMachineName, const QString &strIfNames);

        /** Confirms capturing keyboard/mouse input. */
        static bool confirmInputCapture(bool &fAutoConfirmed);

        /** Confirms going full-screen mode. */
        static bool confirmGoingFullscreen(const QString &strHotKey);
        /** Confirms going seamless mode. */
        static bool confirmGoingSeamless(const QString &strHotKey);
        /** Confirms going scale mode. */
        static bool confirmGoingScale(const QString &strHotKey);

        /** Confirms going full-screen mode anyway.
          * @param  uMinVRAM  Brings the minimum VRAM amount required. */
        static bool confirmGoingFullscreenAnyway(quint64 uMinVRAM);
        /** Confirms switching full-screen mode.
          * @param  uMinVRAM  Brings the minimum VRAM amount required. */
        static bool confirmSwitchingScreenInFullscreen(quint64 uMinVRAM);
    /** @} */

protected:

    /** Constructs question notification-object.
      * @param  strName          Brings the question name.
      * @param  strDetails       Brings the question details.
      * @param  buttonNames      Brings the list of button names.
      * @param  fOkByDefault     Brings whether Ok button should be default one.
      * @param  strInternalName  Brings the question internal name.
      * @param  strHelpKeyword   Brings the question help keyword. */
    UINotificationQuestion(const QString &strName,
                           const QString &strDetails,
                           const QStringList &buttonNames,
                           bool fOkByDefault,
                           const QString &strInternalName,
                           const QString &strHelpKeyword);
    /** Destructs question notification-object. */
    virtual ~UINotificationQuestion() RT_OVERRIDE RT_FINAL;

private:

    /** Creates question.
      * @param  pParent          Brings the local notification-center reference.
      * @param  strName          Brings the question name.
      * @param  strDetails       Brings the question details.
      * @param  buttonNames      Brings the list of button names.
      * @param  fOkByDefault     Brings whether Ok button should be default one.
      * @param  strInternalName  Brings the question internal name.
      * @param  strHelpKeyword   Brings the question help keyword. */
    static void createQuestionInt(UINotificationCenter *pParent,
                                  const QString &strName,
                                  const QString &strDetails,
                                  const QStringList &buttonNames,
                                  bool fOkByDefault,
                                  const QString &strInternalName,
                                  const QString &strHelpKeyword);
    /** Creates blocking question.
      * @param  pParent          Brings the local notification-center reference.
      * @param  strName          Brings the question name.
      * @param  strDetails       Brings the question details.
      * @param  buttonNames      Brings the list of button names.
      * @param  fOkByDefault     Brings whether Ok button should be default one.
      * @param  strInternalName  Brings the question internal name.
      * @param  strHelpKeyword   Brings the question help keyword. */
    static int createBlockingQuestionInt(UINotificationCenter *pParent,
                                         const QString &strName,
                                         const QString &strDetails,
                                         const QStringList &buttonNames,
                                         bool fOkByDefault,
                                         const QString &strInternalName,
                                         const QString &strHelpKeyword);

    /** Creates question.
      * @param  strName          Brings the question name.
      * @param  strDetails       Brings the question details.
      * @param  buttonNames      Brings the list of button names.
      * @param  fOkByDefault     Brings whether Ok button should be default one.
      * @param  strInternalName  Brings the question internal name.
      * @param  strHelpKeyword   Brings the question help keyword.
      * @param  pParent          Brings the local notification-center reference. */
    static void createQuestion(const QString &strName,
                               const QString &strDetails,
                               const QStringList &buttonNames = QStringList(),
                               bool fOkByDefault = true,
                               const QString &strInternalName = QString(),
                               const QString &strHelpKeyword = QString(),
                               QWidget *pParent = 0);
    /** Creates blocking question.
      * @param  strName          Brings the question name.
      * @param  strDetails       Brings the question details.
      * @param  buttonNames      Brings the list of button names.
      * @param  fOkByDefault     Brings whether Ok button should be default one.
      * @param  strInternalName  Brings the question internal name.
      * @param  strHelpKeyword   Brings the question help keyword.
      * @param  pParent          Brings the local notification-center reference. */
    static int createBlockingQuestion(const QString &strName,
                                      const QString &strDetails,
                                      const QStringList &buttonNames = QStringList(),
                                      bool fOkByDefault = true,
                                      const QString &strInternalName = QString(),
                                      const QString &strHelpKeyword = QString(),
                                      QWidget *pParent = 0);

    /** Destroys question.
      * @param  strInternalName  Brings the question internal name.
      * @param  pParent          Brings the local notification-center reference. */
    static void destroyQuestion(const QString &strInternalName,
                                UINotificationCenter *pParent = 0);

    /** Holds the IDs of questions registered. */
    static QMap<QString, QUuid>  m_questions;

    /** Holds the button names. */
    QStringList  m_buttonNames;
    /** Holds whether Ok button should be default one. */
    bool         m_fOkByDefault;

    /** Holds the question result. */
    Question::Result  m_enmResult;
    /** Holds whether current question is done. */
    bool              m_fDone;
};

#endif /* !FEQT_INCLUDED_SRC_notificationcenter_UINotificationQuestion_h */
