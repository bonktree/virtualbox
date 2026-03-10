/* $Id: UINotificationObjectItem.h 113301 2026-03-10 11:21:14Z sergey.dubov@oracle.com $ */
/** @file
 * VBox Qt GUI - UINotificationObjectItem class declaration.
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

#ifndef FEQT_INCLUDED_SRC_notificationcenter_UINotificationObjectItem_h
#define FEQT_INCLUDED_SRC_notificationcenter_UINotificationObjectItem_h
#ifndef RT_WITHOUT_PRAGMA_ONCE
# pragma once
#endif

/* Qt includes: */
#include <QWidget>

/* VBox includes: */
#include <iprt/cdefs.h> // for RT_OVERRIDE stuff

/* Forward declarations: */
class QAbstractButton;
class QHBoxLayout;
class QLabel;
class QProgressBar;
class QPushButton;
class QVBoxLayout;
class QIDialogButtonBox;
class QIRichTextLabel;
class QIToolButton;
class UINotificationMessage;
class UINotificationObject;
class UINotificationProgress;
class UINotificationQuestion;
#ifdef VBOX_GUI_WITH_NETWORK_MANAGER
class UINotificationDownloader;
#endif

/** QWidget-based notification-object item. */
class UINotificationObjectItem : public QWidget
{
    Q_OBJECT;

public:

    /** Constructs notification-object item, passing @a pParent to the base-class.
      * @param  pObject  Brings the notification-object this item created for. */
    UINotificationObjectItem(QWidget *pParent,
                             UINotificationObject *pObject);

    /** Returns notification-object this item created for. */
    UINotificationObject *internalObject() const { return m_pObject; }
    /** Returns whether notification is of critical type. */
    bool isCritical() const;

    /** Prepares everything. */
    void prepare();

    /** Defines details label width @a iHint. */
    void setDetailsWidthHint(int iHint);
    /** Returns details label width hint. */
    int detailsWidthHint() const;

protected slots:

    /** Handles translation event. */
    virtual void sltRetranslateUI();

protected:

    /** Prepares widgets. */
    virtual void prepareWidgets();
    /** Prepares connections. */
    virtual void prepareConnections();

    /** Handles any Qt @a pEvent. */
    virtual bool event(QEvent *pEvent) RT_OVERRIDE;

    /** Handles paint @a pEvent. */
    virtual void paintEvent(QPaintEvent *pEvent) RT_OVERRIDE;

    /** Returns forget-control width-hint. */
    virtual int widthHintForgetControl() const { return 0; }

    /** Holds the notification-object this item created for. */
    UINotificationObject *m_pObject;

    /** Holds the minimum width hint. */
    int  m_iMinimumWidthHint;
    /** Holds the details width hint. */
    int  m_iDetailsWidthHint;

    /** Holds the main layout instance. */
    QVBoxLayout     *m_pLayoutMain;
    /** Holds the upper layout instance. */
    QHBoxLayout     *m_pLayoutUpper;
    /** Holds the name label instance. */
    QLabel          *m_pLabelName;
    /** Holds the help button instance. */
    QIToolButton    *m_pButtonHelp;
    /** Holds the close button instance. */
    QIToolButton    *m_pButtonClose;
    /** Holds the details label instance. */
    QIRichTextLabel *m_pLabelDetails;

    /** Holds whether item is hovered. */
    bool  m_fHovered;
    /** Holds whether item is toggled. */
    bool  m_fToggled;

private slots:

    /** Handles help request. */
    void sltHandleHelpRequest();
};

/** UINotificationObjectItem extension for notification-message. */
class UINotificationMessageItem : public UINotificationObjectItem
{
    Q_OBJECT;

public:

    /** Constructs notification-message item, passing @a pParent to the base-class.
      * @param  pObject  Brings the notification-object this item created for. */
    UINotificationMessageItem(QWidget *pParent,
                              UINotificationObject *pObject);

protected slots:

    /** Handles translation event. */
    virtual void sltRetranslateUI() RT_OVERRIDE RT_FINAL;

protected:

    /** Prepares widgets. */
    virtual void prepareWidgets() RT_OVERRIDE RT_FINAL;
    /** Prepares connections. */
    virtual void prepareConnections() RT_OVERRIDE RT_FINAL;

    /** Returns forget-control width-hint. */
    virtual int widthHintForgetControl() const RT_OVERRIDE RT_FINAL;

private:

    /** Holds the notification-message this item created for. */
    UINotificationMessage *message() const;

    /** Holds the forget button instance. */
    QPushButton *m_pButtonForget;
};

/** UINotificationObjectItem extension for notification-question. */
class UINotificationQuestionItem : public UINotificationObjectItem
{
    Q_OBJECT;

public:

    /** Constructs notification-question item, passing @a pParent to the base-class.
      * @param  pObject  Brings the notification-object this item created for. */
    UINotificationQuestionItem(QWidget *pParent,
                               UINotificationObject *pObject);

protected:

    /** Prepares widgets. */
    virtual void prepareWidgets() RT_OVERRIDE RT_FINAL;
    /** Prepares connections. */
    virtual void prepareConnections() RT_OVERRIDE RT_FINAL;

    /** Handles show @a pEvent. */
    virtual void showEvent(QShowEvent *pEvent) RT_OVERRIDE RT_FINAL;
    /** Handles key-press @a pEvent. */
    virtual void keyPressEvent(QKeyEvent *pEvent) RT_OVERRIDE RT_FINAL;

private slots:

    /** Handles @a pButton click. */
    void sltHandleButtonClick(QAbstractButton *pButton);

private:

    /** Holds the notification-question this item created for. */
    UINotificationQuestion *question() const;

    /** Holds the button-box instance. */
    QIDialogButtonBox *m_pButtonBox;

    /** Holds whether item polished. */
    bool  m_fPolished;
};

/** UINotificationObjectItem extension for notification-progress. */
class UINotificationProgressItem : public UINotificationObjectItem
{
    Q_OBJECT;

public:

    /** Constructs notification-progress item, passing @a pParent to the base-class.
      * @param  pObject  Brings the notification-progress this item created for. */
    UINotificationProgressItem(QWidget *pParent,
                               UINotificationObject *pObject);

protected:

    /** Prepares widgets. */
    virtual void prepareWidgets() RT_OVERRIDE RT_FINAL;
    /** Prepares connections. */
    virtual void prepareConnections() RT_OVERRIDE RT_FINAL;

private slots:

    /** Handles signal about progress started. */
    void sltHandleProgressStarted();
    /** Handles signal about progress changed.
      * @param  uPercent  Brings new progress percentage value. */
    void sltHandleProgressChange(ulong uPercent);
    /** Handles signal about progress finished. */
    void sltHandleProgressFinished();

private:

    /** Holds the notification-progress this item created for. */
    UINotificationProgress *progress() const;

    /** Updates details. */
    void updateDetails();

    /** Holds the progress-bar instance. */
    QProgressBar *m_pProgressBar;
};

#ifdef VBOX_GUI_WITH_NETWORK_MANAGER
/** UINotificationObjectItem extension for notification-downloader. */
class UINotificationDownloaderItem : public UINotificationObjectItem
{
    Q_OBJECT;

public:

    /** Constructs notification-downloader item, passing @a pParent to the base-class.
      * @param  pObject  Brings the notification-downloader this item created for. */
    UINotificationDownloaderItem(QWidget *pParent,
                                 UINotificationObject *pObject);

protected:

    /** Prepares widgets. */
    virtual void prepareWidgets() RT_OVERRIDE RT_FINAL;
    /** Prepares connections. */
    virtual void prepareConnections() RT_OVERRIDE RT_FINAL;

private slots:

    /** Handles signal about progress started. */
    void sltHandleProgressStarted();
    /** Handles signal about progress changed.
      * @param  uPercent  Brings new progress percentage value. */
    void sltHandleProgressChange(ulong uPercent);
    /** Handles signal about progress finished. */
    void sltHandleProgressFinished();

private:

    /** Holds the notification-downloader this item created for. */
    UINotificationDownloader *downloader() const;

    /** Updates details. */
    void updateDetails();

    /** Holds the progress-bar instance. */
    QProgressBar *m_pProgressBar;
};
#endif /* VBOX_GUI_WITH_NETWORK_MANAGER */

/** Notification-object factory. */
namespace UINotificationItem
{
    /** Creates notification-object of required type.
      * @param  pParent    Brings the parent constructed item being attached to.
      * @param  pObject    Brings the notification-object item being constructed for. */
    UINotificationObjectItem *create(QWidget *pParent,
                                     UINotificationObject *pObject);
}

#endif /* !FEQT_INCLUDED_SRC_notificationcenter_UINotificationObjectItem_h */
