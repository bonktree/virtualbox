/* $Id: UIExtraDataManagerWindow.h 112953 2026-02-11 14:30:58Z sergey.dubov@oracle.com $ */
/** @file
 * VBox Qt GUI - UIExtraDataManagerWindow class declaration.
 */

/*
 * Copyright (C) 2010-2026 Oracle and/or its affiliates.
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

#ifndef FEQT_INCLUDED_SRC_extradata_UIExtraDataManagerWindow_h
#define FEQT_INCLUDED_SRC_extradata_UIExtraDataManagerWindow_h
#ifndef RT_WITHOUT_PRAGMA_ONCE
# pragma once
#endif

/* Qt includes: */
#include <QComboBox>
#include <QLineEdit>
#include <QModelIndex>
#include <QSize>
#include <QSortFilterProxyModel>
#include <QString>
#include <QStyledItemDelegate>
#include <QUuid>

/* GUI includes: */
#include "QIDialog.h"
#include "QIMainWindow.h"

/* Forward declarations: */
class QAction;
class QItemSelection;
class QListView;
class QObject;
class QPainter;
class QPixmap;
class QPoint;
class QStandardItem;
class QStandardItemModel;
class QStyleOptionViewItem;
class QTableView;
class QVBoxLayout;
class QIDialogButtonBox;
class QISplitter;
class QIToolBar;
class CMachine;

/** Data fields. */
enum Field
{
    Field_ID = Qt::UserRole + 1,
    Field_Name,
    Field_OsTypeID,
    Field_Known
};

/** QStyledItemDelegate extension
  * reflecting items of Extra Data Manager window: Chooser pane. */
class UIChooserPaneDelegate : public QStyledItemDelegate
{
    Q_OBJECT;

public:

    /** Constructs Chooser pane delegate passing @a pParent to the base-class. */
    UIChooserPaneDelegate(QObject *pParent);

private:

    /** Returns this item's preferred size.
      * @param  option  Brings the option set for current model index.
      * @param  index   Brings currently selected model index. */
    virtual QSize sizeHint(const QStyleOptionViewItem &option, const QModelIndex &index) const RT_OVERRIDE RT_FINAL;

    /** Paints @a index item with specified @a option using specified @a pPainter. */
    virtual void paint(QPainter *pPainter, const QStyleOptionViewItem &option, const QModelIndex &index) const RT_OVERRIDE RT_FINAL;

    /** Fetch pixmap info for passed QModelIndex. */
    static void fetchPixmapInfo(const QModelIndex &index, QPixmap &pixmap, QSize &pixmapSize);

    /** Holds the item margin. */
    int  m_iMargin;
    /** Holds the item spacing. */
    int  m_iSpacing;
};

/** QSortFilterProxyModel extension
  * used by the chooser-pane of the UIExtraDataManagerWindow. */
class UIChooserPaneSortingModel : public QSortFilterProxyModel
{
    Q_OBJECT;

public:

    /** Constructs proxy model passing @a pParent to the base-class. */
    UIChooserPaneSortingModel(QObject *pParent);

protected:

    /** Returns true if the value of the item referred to by the given index left is less than
      * the value of the item referred to by the given index right, otherwise returns false. */
    virtual bool lessThan(const QModelIndex &leftIdx, const QModelIndex &rightIdx) const RT_OVERRIDE RT_FINAL;
};

/** QIDialog extension
  * used for adding new extra-data record. */
class UIAddExtraDataRecordDialog : public QIDialog
{
    Q_OBJECT;

public:

    /** Constructs add new extra-data record dislog passing @a pParent to the base-class. */
    UIAddExtraDataRecordDialog(QWidget *pParent);

    /** Returns key. */
    QString key() const;
    /** Returns value. */
    QString value() const;

private:

    /** Prepares everything. */
    void prepare();

    /** Lists known extra-data keys. */
    static QStringList knownExtraDataKeys();

    /** Holds the key editor instance. */
    QComboBox *m_pEditorKey;
    /** Holds the value editor instance. */
    QLineEdit *m_pEditorValue;
};

/** QIMainWindow extension
  * providing Extra Data Manager with UI features. */
class UIExtraDataManagerWindow : public QIMainWindow
{
    Q_OBJECT;

public:

    /** @name Constructor/Destructor
      * @{ */
        /** Constructs Extra-data Manager window.
          * @param  pCenterWidget  Brings the widget to center window accordingly. */
        UIExtraDataManagerWindow(QWidget *pCenterWidget);
        /** Destructs Extra-data Manager window. */
        virtual ~UIExtraDataManagerWindow() RT_OVERRIDE RT_FINAL;
    /** @} */

    /** @name Management
      * @{ */
        /** Shows and raises the dialog. */
        void showAndRaise();
    /** @} */

public slots:

    /** @name General
      * @{ */
        /** Handles extra-data map acknowledging. */
        void sltExtraDataMapAcknowledging(const QUuid &uID);
        /** Handles extra-data change. */
        void sltExtraDataChange(const QUuid &uID, const QString &strKey, const QString &strValue);
    /** @} */

private slots:

    /** @name General
      * @{ */
        /** Handles machine (un)registration. */
        void sltMachineRegistered(const QUuid &uID, bool fAdded);
    /** @} */

    /** @name Chooser-pane
      * @{ */
        /** Handles filter-apply signal for the chooser-pane. */
        void sltChooserApplyFilter(const QString &strFilter);
        /** Handles current-changed signal for the chooser-pane: */
        void sltChooserHandleCurrentChanged(const QModelIndex &index);
        /** Handles item-selection-changed signal for the chooser-pane: */
        void sltChooserHandleSelectionChanged(const QItemSelection &selected,
                                              const QItemSelection &deselected);
    /** @} */

    /** @name Data-pane
      * @{ */
        /** Handles filter-apply signal for the data-pane. */
        void sltDataApplyFilter(const QString &strFilter);
        /** Handles item-selection-changed signal for the data-pane: */
        void sltDataHandleSelectionChanged(const QItemSelection &selected,
                                           const QItemSelection &deselected);
        /** Handles item-changed signal for the data-pane: */
        void sltDataHandleItemChanged(QStandardItem *pItem);
        /** Handles context-menu-requested signal for the data-pane: */
        void sltDataHandleCustomContextMenuRequested(const QPoint &pos);
    /** @} */

    /** @name Actions
      * @{ */
        /** Add handler. */
        void sltAdd();
        /** Remove handler. */
        void sltDel();
        /** Save handler. */
        void sltSave();
        /** Load handler. */
        void sltLoad();
    /** @} */

private:

    /** @name General
      * @{ */
        /** Returns whether the window should be maximized when geometry being restored. */
        virtual bool shouldBeMaximized() const RT_OVERRIDE;
    /** @} */

    /** @name Prepare/Cleanup
      * @{ */
        /** Prepare instance. */
        void prepare();
        /** Prepare this. */
        void prepareThis();
        /** Prepare connections. */
        void prepareConnections();
        /** Prepare menu. */
        void prepareMenu();
        /** Prepare central widget. */
        void prepareCentralWidget();
        /** Prepare tool-bar. */
        void prepareToolBar();
        /** Prepare splitter. */
        void prepareSplitter();
        /** Prepare panes: */
        void preparePanes();
        /** Prepare chooser pane. */
        void preparePaneChooser();
        /** Prepare data pane. */
        void preparePaneData();
        /** Prepare button-box. */
        void prepareButtonBox();
        /** Load window settings. */
        void loadSettings();

        /** Save window settings. */
        void saveSettings();
        /** Cleanup instance. */
        void cleanup();
    /** @} */

    /** @name Actions
      * @{ */
        /** Updates action availability. */
        void updateActionsAvailability();
    /** @} */

    /** @name Chooser-pane
      * @{ */
        /** Returns chooser index for @a iRow. */
        QModelIndex chooserIndex(int iRow) const;
        /** Returns current chooser index. */
        QModelIndex currentChooserIndex() const;

        /** Returns chooser ID for @a iRow. */
        QUuid chooserID(int iRow) const;
        /** Returns current chooser ID. */
        QUuid currentChooserID() const;

        /** Returns chooser Name for @a iRow. */
        QString chooserName(int iRow) const;
        /** Returns current Name. */
        QString currentChooserName() const;

        /** Adds chooser item. */
        void addChooserItem(const QUuid &uID,
                            const QString &strName,
                            const QString &strOsTypeID,
                            const int iPosition = -1);
        /** Adds chooser item by machine. */
        void addChooserItemByMachine(const CMachine &machine,
                                     const int iPosition = -1);
        /** Adds chooser item by ID. */
        void addChooserItemByID(const QUuid &uID,
                                const int iPosition = -1);

        /** Make sure chooser have current-index if possible. */
        void makeSureChooserHaveCurrentIndexIfPossible();
    /** @} */

    /** @name Data-pane
      * @{ */
        /** Returns data index for @a iRow and @a iColumn. */
        QModelIndex dataIndex(int iRow, int iColumn) const;

        /** Returns data-key index for @a iRow. */
        QModelIndex dataKeyIndex(int iRow) const;

        /** Returns data-value index for @a iRow. */
        QModelIndex dataValueIndex(int iRow) const;

        /** Returns current data-key. */
        QString dataKey(int iRow) const;

        /** Returns current data-value. */
        QString dataValue(int iRow) const;

        /** Adds data item. */
        void addDataItem(const QString &strKey,
                         const QString &strValue,
                         const int iPosition = -1);

        /** Sorts data items. */
        void sortData();
    /** @} */

    /** @name Arguments
      * @{ */
        /** Holds the center widget reference. */
        QWidget *m_pCenterWidget;
    /** @} */

    /** @name General
      * @{ */
        QVBoxLayout *m_pMainLayout;
        /** Data pane: Tool-bar. */
        QIToolBar   *m_pToolBar;
        /** Splitter. */
        QISplitter  *m_pSplitter;
    /** @} */

    /** @name Chooser-pane
      * @{ */
        /** Chooser pane. */
        QWidget   *m_pPaneOfChooser;
        /** Chooser filter. */
        QLineEdit *m_pFilterOfChooser;
        /** Chooser pane: List-view. */
        QListView *m_pViewOfChooser;

        /** Chooser pane: Source-model. */
        QStandardItemModel        *m_pModelSourceOfChooser;
        /** Chooser pane: Proxy-model. */
        UIChooserPaneSortingModel *m_pModelProxyOfChooser;
    /** @} */

    /** @name Data-pane
      * @{ */
        /** Data pane. */
        QWidget    *m_pPaneOfData;
        /** Data filter. */
        QLineEdit  *m_pFilterOfData;
        /** Data pane: Table-view. */
        QTableView *m_pViewOfData;

        /** Data pane: Item-model. */
        QStandardItemModel    *m_pModelSourceOfData;
        /** Data pane: Proxy-model. */
        QSortFilterProxyModel *m_pModelProxyOfData;
    /** @} */

    /** @name Button Box
      * @{ */
        /** Dialog button-box. */
        QIDialogButtonBox *m_pButtonBox;
    /** @} */

    /** @name Actions
      * @{ */
        /** Add action. */
        QAction *m_pActionAdd;
        /** Del action. */
        QAction *m_pActionDel;
        /** Load action. */
        QAction *m_pActionLoad;
        /** Save action. */
        QAction *m_pActionSave;
    /** @} */
};

#endif /* !FEQT_INCLUDED_SRC_extradata_UIExtraDataManagerWindow_h */
