/* $Id: UINotificationQuestion.h 113077 2026-02-18 18:28:10Z sergey.dubov@oracle.com $ */
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
#include "UINotificationObject.h"

/* Forward declarations: */
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

    /** Returns whether object is done. */
    virtual bool isDone() const RT_OVERRIDE RT_FINAL { return m_fDone; }

    /** Returns the button names. */
    QStringList buttonNames() const { return m_buttonNames; }

    /** Returns the result. */
    Question::Result result() const { return m_enmResult; }
    /** Defines the @a enmResult. */
    void setResult(Question::Result enmResult) { m_enmResult = enmResult; m_fDone = true; }

protected:

    /** Constructs question notification-object.
      * @param  strName          Brings the question name.
      * @param  strDetails       Brings the question details.
      * @param  buttonNames      Brings the list of button names.
      * @param  strInternalName  Brings the question internal name.
      * @param  strHelpKeyword   Brings the question help keyword. */
    UINotificationQuestion(const QString &strName,
                           const QString &strDetails,
                           const QStringList &buttonNames,
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
      * @param  strInternalName  Brings the question internal name.
      * @param  strHelpKeyword   Brings the question help keyword. */
    static void createQuestionInt(UINotificationCenter *pParent,
                                  const QString &strName,
                                  const QString &strDetails,
                                  const QStringList &buttonNames,
                                  const QString &strInternalName,
                                  const QString &strHelpKeyword);
    /** Creates blocking question.
      * @param  pParent          Brings the local notification-center reference.
      * @param  strName          Brings the question name.
      * @param  strDetails       Brings the question details.
      * @param  buttonNames      Brings the list of button names.
      * @param  strInternalName  Brings the question internal name.
      * @param  strHelpKeyword   Brings the question help keyword. */
    static int createBlockingQuestionInt(UINotificationCenter *pParent,
                                         const QString &strName,
                                         const QString &strDetails,
                                         const QStringList &buttonNames,
                                         const QString &strInternalName,
                                         const QString &strHelpKeyword);

    /** Creates question.
      * @param  strName     Brings the question name.
      * @param  strDetails  Brings the question details.
      * @param  buttonNames  Brings the list of button names.
      * @param  pParent     Brings the parent reference. */
    static void createQuestion(const QString &strName,
                               const QString &strDetails,
                               const QStringList &buttonNames,
                               QWidget *pParent = 0);
    /** Creates question.
      * @param  strName          Brings the question name.
      * @param  strDetails       Brings the question details.
      * @param  buttonNames  Brings the list of button names.
      * @param  strInternalName  Brings the question internal name.
      * @param  strHelpKeyword   Brings the question help keyword.
      * @param  pParent          Brings the local notification-center reference. */
    static void createQuestion(const QString &strName,
                               const QString &strDetails,
                               const QStringList &buttonNames,
                               const QString &strInternalName,
                               const QString &strHelpKeyword = QString(),
                               QWidget *pParent = 0);
    /** Creates blocking question.
      * @param  strName      Brings the question name.
      * @param  strDetails   Brings the question details.
      * @param  buttonNames  Brings the list of button names.
      * @param  pParent      Brings the parent reference. */
    static int createBlockingQuestion(const QString &strName,
                                      const QString &strDetails,
                                      const QStringList &buttonNames,
                                      QWidget *pParent = 0);
    /** Creates blocking question.
      * @param  strName          Brings the question name.
      * @param  strDetails       Brings the question details.
      * @param  buttonNames      Brings the list of button names.
      * @param  strInternalName  Brings the question internal name.
      * @param  strHelpKeyword   Brings the question help keyword.
      * @param  pParent          Brings the local notification-center reference. */
    static int createBlockingQuestion(const QString &strName,
                                      const QString &strDetails,
                                      const QStringList &buttonNames,
                                      const QString &strInternalName,
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

    /** Holds the question result. */
    Question::Result  m_enmResult;
    /** Holds whether current question is done. */
    bool              m_fDone;
};

#endif /* !FEQT_INCLUDED_SRC_notificationcenter_UINotificationQuestion_h */
