/* $Id: UIExtraDataManagerWindow.cpp 112952 2026-02-11 14:20:19Z sergey.dubov@oracle.com $ */
/** @file
 * VBox Qt GUI - UIExtraDataManagerWindow class implementation.
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

/* Qt includes: */
#include <QApplication>
# include <QComboBox>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QListView>
#include <QMenu>
#include <QMenuBar>
#include <QPainter>
#include <QPushButton>
#include <QRegularExpression>
#include <QStandardItem>
#include <QStandardItemModel>
#include <QTableView>
#include <QVBoxLayout>
#include <QXmlStreamReader>

/* GUI includes: */
#include "QIDialogButtonBox.h"
#include "QIFileDialog.h"
#include "QISplitter.h"
#include "QIToolBar.h"
#include "QIWidgetValidator.h"
#include "UIExtraDataManager.h"
#include "UIExtraDataManagerWindow.h"
#include "UIGlobalSession.h"
#include "UIIconPool.h"
#include "UIVirtualBoxEventHandler.h"
#include "UILoggingDefs.h"
#include "UIMessageCenter.h"
#include "VBoxUtils.h"

/* COM includes: */
#include "CMachine.h"

/* Other VBox includes: */
#include "iprt/assert.h"

/* Namespaces: */
using namespace UIExtraDataDefs;


/*********************************************************************************************************************************
*   Class UIChooserPaneDelegate implementation.                                                                                  *
*********************************************************************************************************************************/

UIChooserPaneDelegate::UIChooserPaneDelegate(QObject *pParent)
    : QStyledItemDelegate(pParent)
    , m_iMargin(3)
    , m_iSpacing(3)
{
}

QSize UIChooserPaneDelegate::sizeHint(const QStyleOptionViewItem &option, const QModelIndex &index) const
{
    /* Font metrics: */
    const QFontMetrics &fm = option.fontMetrics;
    /* Pixmap: */
    QPixmap pixmap;
    QSize pixmapSize;
    fetchPixmapInfo(index, pixmap, pixmapSize);

    /* Calculate width: */
    const int iWidth = m_iMargin
                     + pixmapSize.width()
                     + 2 * m_iSpacing
                     + qMax(fm.horizontalAdvance(index.data(Field_Name).toString()),
                            fm.horizontalAdvance(index.data(Field_ID).toString()))
                     + m_iMargin;
    /* Calculate height: */
    const int iHeight = m_iMargin
                      + qMax(pixmapSize.height(),
                             fm.height() + m_iSpacing + fm.height())
                      + m_iMargin;

    /* Return result: */
    return QSize(iWidth, iHeight);
}

void UIChooserPaneDelegate::paint(QPainter *pPainter, const QStyleOptionViewItem &option, const QModelIndex &index) const
{
    /* Item rect: */
    const QRect &optionRect = option.rect;
    /* Palette: */
    const QPalette &palette = option.palette;
    /* Font metrics: */
    const QFontMetrics &fm = option.fontMetrics;
    /* Pixmap: */
    QPixmap pixmap;
    QSize pixmapSize;
    fetchPixmapInfo(index, pixmap, pixmapSize);

    /* If item selected: */
    if (option.state & QStyle::State_Selected)
    {
        /* Fill background with selection color: */
        QColor highlight = palette.color(option.state & QStyle::State_Active ?
                                         QPalette::Active : QPalette::Inactive,
                                         QPalette::Highlight);
        QLinearGradient bgGrad(optionRect.topLeft(), optionRect.bottomLeft());
        bgGrad.setColorAt(0, highlight.lighter(120));
        bgGrad.setColorAt(1, highlight);
        pPainter->fillRect(optionRect, bgGrad);
        /* Draw focus frame: */
        QStyleOptionFocusRect focusOption;
        focusOption.rect = optionRect;
        QApplication::style()->drawPrimitive(QStyle::PE_FrameFocusRect, &focusOption, pPainter);
    }

    /* Draw pixmap: */
    const QPoint pixmapOrigin = optionRect.topLeft() +
                                QPoint(m_iMargin, m_iMargin);
    pPainter->drawPixmap(pixmapOrigin, pixmap);

    /* Is that known item? */
    bool fKnown = index.data(Field_Known).toBool();
    if (fKnown)
    {
        pPainter->save();
        QFont font = pPainter->font();
        font.setBold(true);
        pPainter->setFont(font);
    }

    /* Draw item name: */
    const QPoint nameOrigin = pixmapOrigin +
                              QPoint(pixmapSize.width(), 0) +
                              QPoint(2 * m_iSpacing, 0) +
                              QPoint(0, fm.ascent());
    pPainter->drawText(nameOrigin, index.data(Field_Name).toString());

    /* Was that known item? */
    if (fKnown)
        pPainter->restore();

    /* Draw item ID: */
    const QPoint idOrigin = nameOrigin +
                            QPoint(0, m_iSpacing) +
                            QPoint(0, fm.height());
    pPainter->drawText(idOrigin, index.data(Field_ID).toString());
}

/* static */
void UIChooserPaneDelegate::fetchPixmapInfo(const QModelIndex &index, QPixmap &pixmap, QSize &pixmapSize)
{
    /* If proper machine ID passed => return corresponding pixmap/size: */
    if (index.data(Field_ID).toUuid() != UIExtraDataManager::GlobalID)
        pixmap = generalIconPool().guestOSTypePixmapDefault(index.data(Field_OsTypeID).toString(), &pixmapSize);
    else
    {
        /* For global ID we return static pixmap/size: */
        const QIcon icon = UIIconPool::iconSet(":/edata_global_32px.png");
        pixmapSize = icon.availableSizes().value(0, QSize(32, 32));
        pixmap = icon.pixmap(pixmapSize);
    }
}


/*********************************************************************************************************************************
*   Class UIChooserPaneSortingModel implementation.                                                                              *
*********************************************************************************************************************************/

UIChooserPaneSortingModel::UIChooserPaneSortingModel(QObject *pParent)
    : QSortFilterProxyModel(pParent)
{
}

bool UIChooserPaneSortingModel::lessThan(const QModelIndex &leftIdx, const QModelIndex &rightIdx) const
{
    /* Compare by ID first: */
    const QUuid strID1 = leftIdx.data(Field_ID).toUuid();
    const QUuid strID2 = rightIdx.data(Field_ID).toUuid();
    if (strID1 == UIExtraDataManager::GlobalID)
        return true;
    else if (strID2 == UIExtraDataManager::GlobalID)
        return false;
    /* Compare role finally: */
    return QSortFilterProxyModel::lessThan(leftIdx, rightIdx);
}


/*********************************************************************************************************************************
*   Class UIAddExtraDataRecordDialog implementation.                                                                             *
*********************************************************************************************************************************/

UIAddExtraDataRecordDialog::UIAddExtraDataRecordDialog(QWidget *pParent)
    : QIDialog(pParent)
{
    prepare();
}

void UIAddExtraDataRecordDialog::prepare()
{
    /* Configure self: */
    setMinimumWidth(400);

    /* Create main-layout: */
    QVBoxLayout *pMainLayout = new QVBoxLayout(this);
    AssertPtrReturnVoid(pMainLayout);
    {
        /* Create dialog validator group: */
        QObjectValidatorGroup *pValidatorGroup = new QObjectValidatorGroup(this);
        AssertReturnVoid(pValidatorGroup);
        /* Create input-layout: */
        QGridLayout *pInputLayout = new QGridLayout;
        AssertPtrReturnVoid(pInputLayout);
        {
            /* Create key-label: */
            QLabel *pLabelKey = new QLabel("&Name:");
            {
                /* Configure key-label: */
                pLabelKey->setAlignment(Qt::AlignRight);
                /* Add key-label into input-layout: */
                pInputLayout->addWidget(pLabelKey, 0, 0);
            }
            /* Create key-editor: */
            QComboBox *pEditorKey = new QComboBox;
            {
                /* Configure key-editor: */
                pEditorKey->setEditable(true);
                pEditorKey->addItems(knownExtraDataKeys());
                pLabelKey->setBuddy(pEditorKey);
                /* Create key-editor property setter: */
                QObjectPropertySetter *pKeyPropertySetter = new QObjectPropertySetter(this, "Key");
                AssertPtrReturnVoid(pKeyPropertySetter);
                {
                    /* Configure key-editor property setter: */
                    connect(pEditorKey, &QComboBox::editTextChanged,
                            pKeyPropertySetter, &QObjectPropertySetter::sltAssignProperty);
                }
                /* Create key-editor validator: */
                QObjectValidator *pKeyValidator
                    = new QObjectValidator(new QRegularExpressionValidator(QRegularExpression("[\\s\\S]+"), this));
                AssertPtrReturnVoid(pKeyValidator);
                {
                    /* Configure key-editor validator: */
                    connect(pEditorKey, &QComboBox::editTextChanged,
                            pKeyValidator, &QObjectValidator::sltValidate);
                    /* Add key-editor validator into dialog validator group: */
                    pValidatorGroup->addObjectValidator(pKeyValidator);
                }
                /* Add key-editor into input-layout: */
                pInputLayout->addWidget(pEditorKey, 0, 1);
            }
            /* Create value-label: */
            QLabel *pLabelValue = new QLabel("&Value:");
            {
                /* Configure value-label: */
                pLabelValue->setAlignment(Qt::AlignRight);
                /* Add value-label into input-layout: */
                pInputLayout->addWidget(pLabelValue, 1, 0);
            }
            /* Create value-editor: */
            QLineEdit *pEditorValue = new QLineEdit;
            {
                /* Configure value-editor: */
                pLabelValue->setBuddy(pEditorValue);
                /* Create value-editor property setter: */
                QObjectPropertySetter *pValuePropertySetter = new QObjectPropertySetter(this, "Value");
                AssertPtrReturnVoid(pValuePropertySetter);
                {
                    /* Configure value-editor property setter: */
                    connect(pEditorValue, &QLineEdit::textEdited,
                            pValuePropertySetter, &QObjectPropertySetter::sltAssignProperty);
                }
                /* Create value-editor validator: */
                QObjectValidator *pValueValidator
                    = new QObjectValidator(new QRegularExpressionValidator(QRegularExpression("[\\s\\S]+"), this));
                AssertPtrReturnVoid(pValueValidator);
                {
                    /* Configure value-editor validator: */
                    connect(pEditorValue, &QLineEdit::textEdited,
                            pValueValidator, &QObjectValidator::sltValidate);
                    /* Add value-editor validator into dialog validator group: */
                    pValidatorGroup->addObjectValidator(pValueValidator);
                }
                /* Add value-editor into input-layout: */
                pInputLayout->addWidget(pEditorValue, 1, 1);
            }
            /* Add input-layout into main-layout: */
            pMainLayout->addLayout(pInputLayout);
        }
        /* Create stretch: */
        pMainLayout->addStretch();
        /* Create dialog button-box: */
        QIDialogButtonBox *pButtonBox = new QIDialogButtonBox;
        AssertPtrReturnVoid(pButtonBox);
        {
            /* Configure button-box: */
            pButtonBox->setStandardButtons(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
            pButtonBox->button(QDialogButtonBox::Ok)->setAutoDefault(true);
            pButtonBox->button(QDialogButtonBox::Ok)->setEnabled(pValidatorGroup->result());
            pButtonBox->button(QDialogButtonBox::Cancel)->setShortcut(Qt::Key_Escape);
            connect(pValidatorGroup, &QObjectValidatorGroup::sigValidityChange,
                    pButtonBox->button(QDialogButtonBox::Ok), &QPushButton::setEnabled);
            connect(pButtonBox, &QIDialogButtonBox::accepted, this, &QIDialog::accept);
            connect(pButtonBox, &QIDialogButtonBox::rejected, this, &QIDialog::reject);
            /* Add button-box into main-layout: */
            pMainLayout->addWidget(pButtonBox);
        }
    }

    /* Apply language settings: */
    setWindowTitle("Add extra-data record..");
}

/* static */
QStringList UIAddExtraDataRecordDialog::knownExtraDataKeys()
{
    return QStringList()
           << QString()
           << GUI_RestrictedDialogs
           << GUI_SuppressMessages << GUI_InvertMessageOption
#ifdef VBOX_NOTIFICATION_CENTER_WITH_KEEP_BUTTON
           << GUI_NotificationCenter_KeepSuccessfullProgresses
#endif
           << GUI_NotificationCenter_Alignment
           << GUI_NotificationCenter_Order
           << GUI_PreventBetaLabel
#ifdef VBOX_GUI_WITH_NETWORK_MANAGER
           << GUI_PreventApplicationUpdate << GUI_UpdateDate << GUI_UpdateCheckCount
#endif
           << GUI_Progress_LegacyMode
           << GUI_Customizations
           << GUI_RestrictedGlobalSettingsPages << GUI_RestrictedMachineSettingsPages
           << GUI_LanguageID
           << GUI_ActivateHoveredMachineWindow
           << GUI_DisableHostScreenSaver
           << GUI_Input_SelectorShortcuts << GUI_Input_MachineShortcuts
           << GUI_RecentFolderHD << GUI_RecentFolderCD << GUI_RecentFolderFD
           << GUI_VISOCreator_RecentFolder << GUI_VISOCreator_DialogGeometry
           << GUI_RecentListHD << GUI_RecentListCD << GUI_RecentListFD
           << GUI_RestrictedNetworkAttachmentTypes
           << GUI_LastSelectorWindowPosition << GUI_SplitterSizes
           << GUI_Toolbar << GUI_Toolbar_Text
           << GUI_Toolbar_MachineTools_Order << GUI_Toolbar_GlobalTools_Order
           << GUI_Tools_LastItemsSelected << GUI_Tools_Detached
           << GUI_Statusbar
           << GUI_GroupDefinitions << GUI_LastItemSelected
           << GUI_Details_Elements
           << GUI_Details_Elements_Preview_UpdateInterval
           << GUI_SnapshotManager_Details_Expanded
           << GUI_VirtualMediaManager_Details_Expanded
           << GUI_HostNetworkManager_Details_Expanded
           << GUI_CloudProfileManager_Restrictions
           << GUI_CloudProfileManager_Details_Expanded
           << GUI_CloudConsoleManager_Restrictions
           << GUI_CloudConsoleManager_Details_Expanded
           << GUI_CloudConsole_PublicKey_Path
           << GUI_HideFromManager << GUI_HideDetails
           << GUI_PreventReconfiguration << GUI_PreventSnapshotOperations
#ifndef VBOX_WS_MAC
           << GUI_MachineWindowIcons << GUI_MachineWindowNamePostfix
#endif
           << GUI_LastNormalWindowPosition << GUI_LastScaleWindowPosition
#ifndef VBOX_WS_MAC
           << GUI_MenuBar_Enabled
#endif
           << GUI_MenuBar_ContextMenu_Enabled
           << GUI_RestrictedRuntimeMenus
           << GUI_RestrictedRuntimeApplicationMenuActions
           << GUI_RestrictedRuntimeMachineMenuActions
           << GUI_RestrictedRuntimeViewMenuActions
           << GUI_RestrictedRuntimeInputMenuActions
           << GUI_RestrictedRuntimeDevicesMenuActions
#ifdef VBOX_WITH_DEBUGGER_GUI
           << GUI_RestrictedRuntimeDebuggerMenuActions
#endif
#ifdef VBOX_WS_MAC
           << GUI_RestrictedRuntimeWindowMenuActions
#endif
           << GUI_RestrictedRuntimeHelpMenuActions
           << GUI_RestrictedVisualStates
           << GUI_Fullscreen << GUI_Seamless << GUI_Scale
#ifdef VBOX_WS_NIX
           << GUI_Fullscreen_LegacyMode
           << GUI_DistinguishMachineWindowGroups
#endif
           << GUI_AutoresizeGuest << GUI_LastVisibilityStatusForGuestScreen << GUI_LastGuestSizeHint
           << GUI_VirtualScreenToHostScreen << GUI_AutomountGuestScreens
#ifndef VBOX_WS_MAC
           << GUI_ShowMiniToolBar << GUI_MiniToolBarAutoHide << GUI_MiniToolBarAlignment
#endif
           << GUI_StatusBar_Enabled << GUI_StatusBar_ContextMenu_Enabled << GUI_RestrictedStatusBarIndicators << GUI_StatusBar_IndicatorOrder
#ifdef VBOX_WS_MAC
           << GUI_RealtimeDockIconUpdateEnabled << GUI_RealtimeDockIconUpdateMonitor << GUI_DockIconDisableOverlay
#endif
           << GUI_PassCAD
           << GUI_MouseCapturePolicy
           << GUI_GuruMeditationHandler
           << GUI_HidLedsSync
           << GUI_ScaleFactor << GUI_Scaling_Optimization
           << GUI_SessionInformationDialogGeometry
           << GUI_GuestControl_ProcessControlSplitterHints
           << GUI_GuestControl_FileManagerDialogGeometry
           << GUI_GuestControl_FileManagerOptions
           << GUI_GuestControl_ProcessControlDialogGeometry
           << GUI_DefaultCloseAction << GUI_RestrictedCloseActions
           << GUI_LastCloseAction << GUI_CloseActionHook << GUI_DiscardStateOnPowerOff
#ifdef VBOX_WITH_DEBUGGER_GUI
           << GUI_Dbg_Enabled << GUI_Dbg_AutoShow
#endif
           << GUI_ExtraDataManager_Geometry << GUI_ExtraDataManager_SplitterHints
           << GUI_LogWindowGeometry
           << GUI_HelpBrowser_LastURLList
           << GUI_HelpBrowser_DialogGeometry
           << GUI_HelpBrowser_Bookmarks
           << GUI_HelpBrowser_ZoomPercentage;
}


/*********************************************************************************************************************************
*   Class UIExtraDataManagerWindow implementation.                                                                               *
*********************************************************************************************************************************/

UIExtraDataManagerWindow::UIExtraDataManagerWindow(QWidget *pCenterWidget)
    : m_pCenterWidget(pCenterWidget)
    , m_pMainLayout(0), m_pToolBar(0), m_pSplitter(0)
    , m_pPaneOfChooser(0), m_pFilterOfChooser(0), m_pViewOfChooser(0)
    , m_pModelSourceOfChooser(0), m_pModelProxyOfChooser(0)
    , m_pPaneOfData(0), m_pFilterOfData(0), m_pViewOfData(0),
      m_pModelSourceOfData(0), m_pModelProxyOfData(0)
    , m_pButtonBox(0)
    , m_pActionAdd(0), m_pActionDel(0)
    , m_pActionLoad(0), m_pActionSave(0)
{
    prepare();
}

UIExtraDataManagerWindow::~UIExtraDataManagerWindow()
{
    cleanup();
}

void UIExtraDataManagerWindow::showAndRaise()
{
    /* Show: */
    show();
    /* Restore from minimized state: */
    setWindowState(windowState() & ~Qt::WindowMinimized);
    /* Raise: */
    activateWindow();
}

void UIExtraDataManagerWindow::sltMachineRegistered(const QUuid &uID, bool fRegistered)
{
    /* Machine registered: */
    if (fRegistered)
    {
        /* Gather list of 'known IDs': */
        QList<QUuid> knownIDs;
        for (int iRow = 0; iRow < m_pModelSourceOfChooser->rowCount(); ++iRow)
            knownIDs.append(chooserID(iRow));

        /* Get machine items: */
        const CMachineVector machines = gpGlobalSession->virtualBox().GetMachines();
        /* Look for the proper place to insert new machine item: */
        QUuid uPositionID = UIExtraDataManager::GlobalID;
        foreach (const CMachine &machine, machines)
        {
            /* Get iterated machine ID: */
            const QUuid uIteratedID = machine.GetId();
            /* If 'iterated ID' equal to 'added ID' => break now: */
            if (uIteratedID == uID)
                break;
            /* If 'iterated ID' is 'known ID' => remember it: */
            if (knownIDs.contains(uIteratedID))
                uPositionID = uIteratedID;
        }

        /* Add new chooser item into source-model: */
        addChooserItemByID(uID, knownIDs.indexOf(uPositionID) + 1);
        /* And sort proxy-model: */
        m_pModelProxyOfChooser->sort(0, Qt::AscendingOrder);
        /* Make sure chooser have current-index if possible: */
        makeSureChooserHaveCurrentIndexIfPossible();
    }
    /* Machine unregistered: */
    else
    {
        /* Remove chooser item with 'removed ID' if it is among 'known IDs': */
        for (int iRow = 0; iRow < m_pModelSourceOfChooser->rowCount(); ++iRow)
            if (chooserID(iRow) == uID)
                m_pModelSourceOfChooser->removeRow(iRow);
    }
}

void UIExtraDataManagerWindow::sltExtraDataMapAcknowledging(const QUuid &uID)
{
    /* Update item with 'changed ID' if it is among 'known IDs': */
    for (int iRow = 0; iRow < m_pModelSourceOfChooser->rowCount(); ++iRow)
        if (chooserID(iRow) == uID)
            m_pModelSourceOfChooser->itemFromIndex(chooserIndex(iRow))->setData(true, Field_Known);
}

void UIExtraDataManagerWindow::sltExtraDataChange(const QUuid &uID, const QString &strKey, const QString &strValue)
{
    /* Skip unrelated IDs: */
    if (currentChooserID() != uID)
        return;

    /* List of 'known keys': */
    QStringList knownKeys;
    for (int iRow = 0; iRow < m_pModelSourceOfData->rowCount(); ++iRow)
        knownKeys << dataKey(iRow);

    /* Check if 'changed key' is 'known key': */
    int iPosition = knownKeys.indexOf(strKey);
    /* If that is 'known key': */
    if (iPosition != -1)
    {
        /* If 'changed value' is empty => REMOVE item: */
        if (strValue.isEmpty())
            m_pModelSourceOfData->removeRow(iPosition);
        /* If 'changed value' is NOT empty => UPDATE item: */
        else
        {
            m_pModelSourceOfData->itemFromIndex(dataKeyIndex(iPosition))->setData(strKey, Qt::UserRole);
            m_pModelSourceOfData->itemFromIndex(dataValueIndex(iPosition))->setText(strValue);
        }
    }
    /* Else if 'changed value' is NOT empty: */
    else if (!strValue.isEmpty())
    {
        /* Look for the proper place for 'changed key': */
        QString strPositionKey;
        foreach (const QString &strIteratedKey, gEDataManager->map(uID).keys())
        {
            /* If 'iterated key' equal to 'changed key' => break now: */
            if (strIteratedKey == strKey)
                break;
            /* If 'iterated key' is 'known key' => remember it: */
            if (knownKeys.contains(strIteratedKey))
                strPositionKey = strIteratedKey;
        }
        /* Calculate resulting position: */
        iPosition = knownKeys.indexOf(strPositionKey) + 1;
        /* INSERT item to the required position: */
        addDataItem(strKey, strValue, iPosition);
        /* And sort proxy-model: */
        sortData();
    }
}

void UIExtraDataManagerWindow::sltChooserApplyFilter(const QString &strFilter)
{
    /* Apply filtering rule: */
    m_pModelProxyOfChooser->setFilterWildcard(strFilter);
    /* Make sure chooser have current-index if possible: */
    makeSureChooserHaveCurrentIndexIfPossible();
}

void UIExtraDataManagerWindow::sltChooserHandleCurrentChanged(const QModelIndex &index)
{
    /* Remove all the old items first: */
    while (m_pModelSourceOfData->rowCount())
        m_pModelSourceOfData->removeRow(0);

    /* Ignore invalid indexes: */
    if (!index.isValid())
        return;

    /* Add all the new items finally: */
    const QUuid uID = index.data(Field_ID).toUuid();
    if (!gEDataManager->contains(uID))
        gEDataManager->hotloadMachineExtraDataMap(uID);
    const ExtraDataMap data = gEDataManager->map(uID);
    foreach (const QString &strKey, data.keys())
        addDataItem(strKey, data.value(strKey));
    /* And sort proxy-model: */
    sortData();
}

void UIExtraDataManagerWindow::sltChooserHandleSelectionChanged(const QItemSelection&,
                                                                const QItemSelection&)
{
    /* Update actions availability: */
    updateActionsAvailability();
}

void UIExtraDataManagerWindow::sltDataApplyFilter(const QString &strFilter)
{
    /* Apply filtering rule: */
    m_pModelProxyOfData->setFilterWildcard(strFilter);
}

void UIExtraDataManagerWindow::sltDataHandleSelectionChanged(const QItemSelection&,
                                                             const QItemSelection&)
{
    /* Update actions availability: */
    updateActionsAvailability();
}

void UIExtraDataManagerWindow::sltDataHandleItemChanged(QStandardItem *pItem)
{
    /* Make sure passed item is valid: */
    AssertPtrReturnVoid(pItem);

    /* Item-data index: */
    const QModelIndex itemIndex = m_pModelSourceOfData->indexFromItem(pItem);
    const int iRow = itemIndex.row();
    const int iColumn = itemIndex.column();

    /* Key-data is changed: */
    if (iColumn == 0)
    {
        /* Should we replace changed key? */
        bool fReplace = true;

        /* List of 'known keys': */
        QStringList knownKeys;
        for (int iKeyRow = 0; iKeyRow < m_pModelSourceOfData->rowCount(); ++iKeyRow)
        {
            /* Do not consider the row we are changing as Qt's model is not yet updated: */
            if (iKeyRow != iRow)
                knownKeys << dataKey(iKeyRow);
        }

        /* If changed key exists: */
        if (knownKeys.contains(itemIndex.data().toString()))
        {
            /* Show warning and ask for overwriting approval: */
            if (!msgCenter().questionBinary(this, MessageType_Question,
                                            QString("Overwriting already existing key, Continue?"),
                                            0 /* auto-confirm id */,
                                            QString("Overwrite") /* ok button text */,
                                            QString() /* cancel button text */,
                                            false /* ok button by default? */))
            {
                /* Cancel the operation, restore the original extra-data key: */
                pItem->setData(itemIndex.data(Qt::UserRole).toString(), Qt::DisplayRole);
                fReplace = false;
            }
            else
            {
                /* Delete previous extra-data key: */
                gEDataManager->setExtraDataString(itemIndex.data().toString(),
                                                  QString(),
                                                  currentChooserID());
            }
        }

        /* Replace changed extra-data key if necessary: */
        if (fReplace)
        {
            gEDataManager->setExtraDataString(itemIndex.data(Qt::UserRole).toString(),
                                              QString(),
                                              currentChooserID());
            gEDataManager->setExtraDataString(itemIndex.data().toString(),
                                              dataValue(iRow),
                                              currentChooserID());
        }
    }
    /* Value-data is changed: */
    else
    {
        /* Key-data index: */
        const QModelIndex keyIndex = dataKeyIndex(iRow);
        /* Update extra-data: */
        gEDataManager->setExtraDataString(keyIndex.data().toString(),
                                          itemIndex.data().toString(),
                                          currentChooserID());
    }
}

void UIExtraDataManagerWindow::sltDataHandleCustomContextMenuRequested(const QPoint &pos)
{
    /* Prepare menu: */
    QMenu menu;
    menu.addAction(m_pActionAdd);
    menu.addAction(m_pActionDel);
    menu.addSeparator();
    menu.addAction(m_pActionSave);
    /* Execute menu: */
    m_pActionSave->setProperty("CalledFromContextMenu", true);
    menu.exec(m_pViewOfData->viewport()->mapToGlobal(pos));
    m_pActionSave->setProperty("CalledFromContextMenu", QVariant());
}

void UIExtraDataManagerWindow::sltAdd()
{
    /* Make sure this slot called by corresponding action only: */
    QAction *pSenderAction = qobject_cast<QAction*>(sender());
    AssertReturnVoid(pSenderAction && m_pActionAdd);

    /* Create input-dialog: */
    QPointer<UIAddExtraDataRecordDialog> pInputDialog = new UIAddExtraDataRecordDialog(this);
    AssertPtrReturnVoid(pInputDialog.data());

    /* Execute input-dialog: */
    if (pInputDialog->exec() == QDialog::Accepted)
    {
        /* Should we add new key? */
        bool fAdd = true;

        /* List of 'known keys': */
        QStringList knownKeys;
        for (int iKeyRow = 0; iKeyRow < m_pModelSourceOfData->rowCount(); ++iKeyRow)
            knownKeys << dataKey(iKeyRow);

        /* If new key exists: */
        if (knownKeys.contains(pInputDialog->property("Key").toString()))
        {
            /* Show warning and ask for overwriting approval: */
            if (!msgCenter().questionBinary(this, MessageType_Question,
                                            QString("Overwriting already existing key, Continue?"),
                                            0 /* auto-confirm id */,
                                            QString("Overwrite") /* ok button text */,
                                            QString() /* cancel button text */,
                                            false /* ok button by default? */))
            {
                /* Cancel the operation: */
                fAdd = false;
            }
        }

        /* Add new extra-data key if necessary: */
        if (fAdd)
            gEDataManager->setExtraDataString(pInputDialog->property("Key").toString(),
                                              pInputDialog->property("Value").toString(),
                                              currentChooserID());
    }

    /* Destroy input-dialog: */
    if (pInputDialog)
        delete pInputDialog;
}

void UIExtraDataManagerWindow::sltDel()
{
    /* Make sure this slot called by corresponding action only: */
    QAction *pSenderAction = qobject_cast<QAction*>(sender());
    AssertReturnVoid(pSenderAction && m_pActionDel);

    /* Gather the map of chosen items: */
    QMap<QString, QString> items;
    foreach (const QModelIndex &keyIndex, m_pViewOfData->selectionModel()->selectedRows(0))
        items.insert(keyIndex.data().toString(), dataValueIndex(keyIndex.row()).data().toString());

    /* Prepare details: */
    const QString strTableTemplate("<!--EOM--><table border=0 cellspacing=10 cellpadding=0 width=500>%1</table>");
    const QString strRowTemplate("<tr><td><tt>%1</tt></td><td align=right><tt>%2</tt></td></tr>");
    QString strDetails;
    foreach (const QString &strKey, items.keys())
        strDetails += strRowTemplate.arg(strKey, items.value(strKey));
    strDetails = strTableTemplate.arg(strDetails);

    /* Ask for user' confirmation: */
    if (!msgCenter().errorWithQuestion(this, MessageType_Question,
                                       QString("<p>Do you really wish to "
                                               "remove chosen records?</p>"),
                                       strDetails))
        return;

    /* Erase all the chosen extra-data records: */
    foreach (const QString &strKey, items.keys())
        gEDataManager->setExtraDataString(strKey, QString(), currentChooserID());
}

void UIExtraDataManagerWindow::sltSave()
{
    /* Make sure this slot called by corresponding action only: */
    QAction *pSenderAction = qobject_cast<QAction*>(sender());
    AssertReturnVoid(pSenderAction && m_pActionSave);

    /* Compose initial file-name: */
    const QString strInitialFileName = QDir(gpGlobalSession->homeFolder()).absoluteFilePath(QString("%1_ExtraData.xml").arg(currentChooserName()));
    /* Open file-save dialog to choose file to save extra-data into: */
    const QString strFileName = QIFileDialog::getSaveFileName(strInitialFileName, "XML files (*.xml)", this,
                                                              "Choose file to save extra-data into..", 0, true, true);
    /* Make sure file-name was chosen: */
    if (strFileName.isEmpty())
        return;

    /* Create file: */
    QFile output(strFileName);
    /* Open file for writing: */
    bool fOpened = output.open(QIODevice::WriteOnly);
    AssertReturnVoid(fOpened);
    {
        /* Create XML stream writer: */
        QXmlStreamWriter stream(&output);
        /* Configure XML stream writer: */
        stream.setAutoFormatting(true);
        stream.setAutoFormattingIndent(2);
        /* Write document: */
        stream.writeStartDocument();
        {
            stream.writeStartElement("VirtualBox");
            {
                const QUuid uID = currentChooserID();
                bool fIsMachine = uID != UIExtraDataManager::GlobalID;
                const QString strType = fIsMachine ? "Machine" : "Global";
                stream.writeStartElement(strType);
                {
                    if (fIsMachine)
                        stream.writeAttribute("uuid", QString("{%1}").arg(uID.toString()));
                    stream.writeStartElement("ExtraData");
                    {
                        /* Called from context-menu: */
                        if (pSenderAction->property("CalledFromContextMenu").toBool() &&
                            !m_pViewOfData->selectionModel()->selection().isEmpty())
                        {
                            foreach (const QModelIndex &keyIndex, m_pViewOfData->selectionModel()->selectedRows())
                            {
                                /* Get data-value index: */
                                const QModelIndex valueIndex = dataValueIndex(keyIndex.row());
                                /* Write corresponding extra-data item into stream: */
                                stream.writeStartElement("ExtraDataItem");
                                {
                                    stream.writeAttribute("name", keyIndex.data().toString());
                                    stream.writeAttribute("value", valueIndex.data().toString());
                                }
                                stream.writeEndElement(); /* ExtraDataItem */
                            }
                        }
                        /* Called from menu-bar/tool-bar: */
                        else
                        {
                            for (int iRow = 0; iRow < m_pModelProxyOfData->rowCount(); ++iRow)
                            {
                                /* Get indexes: */
                                const QModelIndex keyIndex = m_pModelProxyOfData->index(iRow, 0);
                                const QModelIndex valueIndex = m_pModelProxyOfData->index(iRow, 1);
                                /* Write corresponding extra-data item into stream: */
                                stream.writeStartElement("ExtraDataItem");
                                {
                                    stream.writeAttribute("name", keyIndex.data().toString());
                                    stream.writeAttribute("value", valueIndex.data().toString());
                                }
                                stream.writeEndElement(); /* ExtraDataItem */
                            }
                        }
                    }
                    stream.writeEndElement(); /* ExtraData */
                }
                stream.writeEndElement(); /* strType */
            }
            stream.writeEndElement(); /* VirtualBox */
        }
        stream.writeEndDocument();
        /* Close file: */
        output.close();
    }
}

void UIExtraDataManagerWindow::sltLoad()
{
    /* Make sure this slot called by corresponding action only: */
    QAction *pSenderAction = qobject_cast<QAction*>(sender());
    AssertReturnVoid(pSenderAction && m_pActionLoad);

    /* Compose initial file-name: */
    const QString strInitialFileName = QDir(gpGlobalSession->homeFolder()).absoluteFilePath(QString("%1_ExtraData.xml").arg(currentChooserName()));
    /* Open file-open dialog to choose file to open extra-data into: */
    const QString strFileName = QIFileDialog::getOpenFileName(strInitialFileName, "XML files (*.xml)", this,
                                                              "Choose file to load extra-data from..");
    /* Make sure file-name was chosen: */
    if (strFileName.isEmpty())
        return;

    /* Create file: */
    QFile input(strFileName);
    /* Open file for writing: */
    bool fOpened = input.open(QIODevice::ReadOnly);
    AssertReturnVoid(fOpened);
    {
        /* Create XML stream reader: */
        QXmlStreamReader stream(&input);
        /* Read XML stream: */
        while (!stream.atEnd())
        {
            /* Read subsequent token: */
            const QXmlStreamReader::TokenType tokenType = stream.readNext();
            /* Skip non-interesting tokens: */
            if (tokenType != QXmlStreamReader::StartElement)
                continue;

            /* Get the name of the current element: */
            const QString strElementName = stream.name().toString();

            /* Search for the scope ID: */
            QUuid uLoadingID;
            if (strElementName == "Global")
                uLoadingID = UIExtraDataManager::GlobalID;
            else if (strElementName == "Machine")
            {
                const QXmlStreamAttributes attributes = stream.attributes();
                if (attributes.hasAttribute("uuid"))
                {
                    const QString strUuid = attributes.value("uuid").toString();
                    const QUuid uLoadingID(strUuid);
                    if (uLoadingID.isNull())
                        msgCenter().alert(this, MessageType_Warning,
                                          QString("<p>Invalid extra-data ID:</p>"
                                                  "<p>%1</p>").arg(strUuid));
                }
            }
            /* Look particular extra-data entries: */
            else if (strElementName == "ExtraDataItem")
            {
                const QXmlStreamAttributes attributes = stream.attributes();
                if (attributes.hasAttribute("name") && attributes.hasAttribute("value"))
                {
                    const QString strName = attributes.value("name").toString();
                    const QString strValue = attributes.value("value").toString();
                    gEDataManager->setExtraDataString(strName, strValue, currentChooserID());
                }
            }

            /* Check extra-data ID: */
            if (!uLoadingID.isNull() && uLoadingID != currentChooserID() &&
                !msgCenter().questionBinary(this, MessageType_Question,
                                            QString("<p>Inconsistent extra-data ID:</p>"
                                                    "<p>Current: {%1}</p>"
                                                    "<p>Loading: {%2}</p>"
                                                    "<p>Continue with loading?</p>")
                                                    .arg(currentChooserID().toString(), uLoadingID.toString())))
                break;
        }
        /* Handle XML stream error: */
        if (stream.hasError())
            msgCenter().alert(this, MessageType_Warning,
                              QString("<p>Error reading XML file:</p>"
                                      "<p>%1</p>").arg(stream.error()));
        /* Close file: */
        input.close();
    }
}

bool UIExtraDataManagerWindow::shouldBeMaximized() const
{
    return gEDataManager->extraDataManagerShouldBeMaximized();
}

void UIExtraDataManagerWindow::prepare()
{
    /* Prepare this: */
    prepareThis();
    /* Prepare connections: */
    prepareConnections();
    /* Prepare menu: */
    prepareMenu();
    /* Prepare central-widget: */
    prepareCentralWidget();
    /* Load settings: */
    loadSettings();
}

void UIExtraDataManagerWindow::prepareThis()
{
#ifndef VBOX_WS_MAC
    /* Assign window icon: */
    setWindowIcon(UIIconPool::iconSetFull(":/edata_manager_32px.png", ":/edata_manager_16px.png"));
#endif

    /* Apply window title: */
    setWindowTitle("Extra-data Manager");

    /* Do not count that window as important for application,
     * it will NOT be taken into account when other top-level windows will be closed: */
    setAttribute(Qt::WA_QuitOnClose, false);

    /* Delete window when closed: */
    setAttribute(Qt::WA_DeleteOnClose);
}

void UIExtraDataManagerWindow::prepareConnections()
{
    /* Prepare connections: */
    connect(gVBoxEvents, &UIVirtualBoxEventHandler::sigMachineRegistered,
            this, &UIExtraDataManagerWindow::sltMachineRegistered);
}

void UIExtraDataManagerWindow::prepareMenu()
{
    /* Create 'Actions' menu: */
    QMenu *pActionsMenu = menuBar()->addMenu("Actions");
    AssertReturnVoid(pActionsMenu);
    {
        /* Create 'Add' action: */
        m_pActionAdd = pActionsMenu->addAction("Add");
        AssertReturnVoid(m_pActionAdd);
        {
            /* Configure 'Add' action: */
            m_pActionAdd->setIcon(UIIconPool::iconSetFull(":/edata_add_24px.png", ":/edata_add_16px.png",
                                                          ":/edata_add_disabled_24px.png", ":/edata_add_disabled_16px.png"));
            m_pActionAdd->setShortcut(QKeySequence("Ctrl+T"));
            connect(m_pActionAdd, &QAction::triggered, this, &UIExtraDataManagerWindow::sltAdd);
        }
        /* Create 'Del' action: */
        m_pActionDel = pActionsMenu->addAction("Remove");
        AssertReturnVoid(m_pActionDel);
        {
            /* Configure 'Del' action: */
            m_pActionDel->setIcon(UIIconPool::iconSetFull(":/edata_remove_24px.png", ":/edata_remove_16px.png",
                                                          ":/edata_remove_disabled_24px.png", ":/edata_remove_disabled_16px.png"));
            m_pActionDel->setShortcut(QKeySequence("Ctrl+R"));
            connect(m_pActionDel, &QAction::triggered, this, &UIExtraDataManagerWindow::sltDel);
        }

        /* Add separator: */
        pActionsMenu->addSeparator();

        /* Create 'Load' action: */
        m_pActionLoad = pActionsMenu->addAction("Load");
        AssertReturnVoid(m_pActionLoad);
        {
            /* Configure 'Load' action: */
            m_pActionLoad->setIcon(UIIconPool::iconSetFull(":/edata_load_24px.png", ":/edata_load_16px.png",
                                                           ":/edata_load_disabled_24px.png", ":/edata_load_disabled_16px.png"));
            m_pActionLoad->setShortcut(QKeySequence("Ctrl+L"));
            connect(m_pActionLoad, &QAction::triggered, this, &UIExtraDataManagerWindow::sltLoad);
        }
        /* Create 'Save' action: */
        m_pActionSave = pActionsMenu->addAction("Save As...");
        AssertReturnVoid(m_pActionSave);
        {
            /* Configure 'Save' action: */
            m_pActionSave->setIcon(UIIconPool::iconSetFull(":/edata_save_24px.png", ":/edata_save_16px.png",
                                                           ":/edata_save_disabled_24px.png", ":/edata_save_disabled_16px.png"));
            m_pActionSave->setShortcut(QKeySequence("Ctrl+S"));
            connect(m_pActionSave, &QAction::triggered, this, &UIExtraDataManagerWindow::sltSave);
        }
    }
}

void UIExtraDataManagerWindow::prepareCentralWidget()
{
    /* Prepare central-widget: */
    setCentralWidget(new QWidget);
    AssertPtrReturnVoid(centralWidget());
    {
        /* Prepare layout: */
        m_pMainLayout = new QVBoxLayout(centralWidget());
        AssertReturnVoid(m_pMainLayout && centralWidget()->layout() &&
                         m_pMainLayout == centralWidget()->layout());
        {
#ifdef VBOX_WS_MAC
            /* No spacing/margins on the Mac: */
            m_pMainLayout->setContentsMargins(0, 0, 0, 0);
            m_pMainLayout->insertSpacing(0, 10);
#else /* !VBOX_WS_MAC */
            /* Set spacing/margin like in the selector window: */
            const int iL = qApp->style()->pixelMetric(QStyle::PM_LayoutLeftMargin) / 2;
            const int iT = qApp->style()->pixelMetric(QStyle::PM_LayoutTopMargin) / 2;
            const int iR = qApp->style()->pixelMetric(QStyle::PM_LayoutRightMargin) / 2;
            const int iB = qApp->style()->pixelMetric(QStyle::PM_LayoutBottomMargin) / 2;
            m_pMainLayout->setContentsMargins(iL, iT, iR, iB);
#endif /* !VBOX_WS_MAC */
            /* Prepare tool-bar: */
            prepareToolBar();
            /* Prepare splitter: */
            prepareSplitter();
            /* Prepare button-box: */
            prepareButtonBox();
        }
        /* Initial focus: */
        if (m_pViewOfChooser)
            m_pViewOfChooser->setFocus();
    }
}

void UIExtraDataManagerWindow::prepareToolBar()
{
    /* Create tool-bar: */
    m_pToolBar = new QIToolBar(this);
    AssertPtrReturnVoid(m_pToolBar);
    {
        /* Configure tool-bar: */
        m_pToolBar->setIconSize(QSize(24, 24));
        m_pToolBar->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
        /* Add actions: */
        m_pToolBar->addAction(m_pActionAdd);
        m_pToolBar->addAction(m_pActionDel);
        m_pToolBar->addSeparator();
        m_pToolBar->addAction(m_pActionLoad);
        m_pToolBar->addAction(m_pActionSave);
        /* Integrate tool-bar into dialog: */
#ifdef VBOX_WS_MAC
        /* Enable unified tool-bars on Mac OS X. Available on Qt >= 4.3: */
        addToolBar(m_pToolBar);
        m_pToolBar->enableMacToolbar();
#else /* !VBOX_WS_MAC */
        /* Add tool-bar into main-layout: */
        m_pMainLayout->addWidget(m_pToolBar);
#endif /* !VBOX_WS_MAC */
    }
}

void UIExtraDataManagerWindow::prepareSplitter()
{
    /* Create splitter: */
    m_pSplitter = new QISplitter;
    AssertPtrReturnVoid(m_pSplitter);
    {
        /* Prepare panes: */
        preparePanes();
        /* Configure splitter: */
        m_pSplitter->setChildrenCollapsible(false);
        m_pSplitter->setStretchFactor(0, 0);
        m_pSplitter->setStretchFactor(1, 1);
        /* Add splitter into main layout: */
        m_pMainLayout->addWidget(m_pSplitter);
    }
}

void UIExtraDataManagerWindow::preparePanes()
{
    /* Prepare chooser-pane: */
    preparePaneChooser();
    /* Prepare data-pane: */
    preparePaneData();
    /* Link chooser and data panes: */
    connect(m_pViewOfChooser->selectionModel(), &QItemSelectionModel::currentChanged,
            this, &UIExtraDataManagerWindow::sltChooserHandleCurrentChanged);
    connect(m_pViewOfChooser->selectionModel(), &QItemSelectionModel::selectionChanged,
            this, &UIExtraDataManagerWindow::sltChooserHandleSelectionChanged);
    connect(m_pViewOfData->selectionModel(), &QItemSelectionModel::selectionChanged,
            this, &UIExtraDataManagerWindow::sltDataHandleSelectionChanged);
    connect(m_pModelSourceOfData, &QStandardItemModel::itemChanged,
            this, &UIExtraDataManagerWindow::sltDataHandleItemChanged);
    /* Make sure chooser have current-index if possible: */
    makeSureChooserHaveCurrentIndexIfPossible();
}

void UIExtraDataManagerWindow::preparePaneChooser()
{
    /* Create chooser-pane: */
    m_pPaneOfChooser = new QWidget;
    AssertPtrReturnVoid(m_pPaneOfChooser);
    {
        /* Create layout: */
        QVBoxLayout *pLayout = new QVBoxLayout(m_pPaneOfChooser);
        AssertReturnVoid(pLayout && m_pPaneOfChooser->layout() &&
                         pLayout == m_pPaneOfChooser->layout());
        {
            /* Configure layout: */
            const int iR = qApp->style()->pixelMetric(QStyle::PM_LayoutRightMargin) / 3;
            pLayout->setContentsMargins(0, 0, iR, 0);
            /* Create chooser-filter: */
            m_pFilterOfChooser = new QLineEdit;
            {
                /* Configure chooser-filter: */
                m_pFilterOfChooser->setPlaceholderText("Search..");
                connect(m_pFilterOfChooser, &QLineEdit::textChanged,
                        this, &UIExtraDataManagerWindow::sltChooserApplyFilter);
                /* Add chooser-filter into layout: */
                pLayout->addWidget(m_pFilterOfChooser);
            }
            /* Create chooser-view: */
            m_pViewOfChooser = new QListView;
            AssertPtrReturnVoid(m_pViewOfChooser);
            {
                /* Configure chooser-view: */
                delete m_pViewOfChooser->itemDelegate();
                m_pViewOfChooser->setItemDelegate(new UIChooserPaneDelegate(m_pViewOfChooser));
                m_pViewOfChooser->setSelectionMode(QAbstractItemView::SingleSelection);
                /* Create source-model: */
                m_pModelSourceOfChooser = new QStandardItemModel(m_pViewOfChooser);
                AssertPtrReturnVoid(m_pModelSourceOfChooser);
                {
                    /* Create proxy-model: */
                    m_pModelProxyOfChooser = new UIChooserPaneSortingModel(m_pViewOfChooser);
                    AssertPtrReturnVoid(m_pModelProxyOfChooser);
                    {
                        /* Configure proxy-model: */
                        m_pModelProxyOfChooser->setSortRole(Field_Name);
                        m_pModelProxyOfChooser->setFilterRole(Field_Name);
                        m_pModelProxyOfChooser->setSortCaseSensitivity(Qt::CaseInsensitive);
                        m_pModelProxyOfChooser->setFilterCaseSensitivity(Qt::CaseInsensitive);
                        m_pModelProxyOfChooser->setSourceModel(m_pModelSourceOfChooser);
                        m_pViewOfChooser->setModel(m_pModelProxyOfChooser);
                    }
                    /* Add global chooser item into source-model: */
                    addChooserItemByID(UIExtraDataManager::GlobalID);
                    /* Add machine chooser items into source-model: */
                    CMachineVector machines = gpGlobalSession->virtualBox().GetMachines();
                    foreach (const CMachine &machine, machines)
                        addChooserItemByMachine(machine);
                    /* And sort proxy-model: */
                    m_pModelProxyOfChooser->sort(0, Qt::AscendingOrder);
                }
                /* Add chooser-view into layout: */
                pLayout->addWidget(m_pViewOfChooser);
            }
        }
        /* Add chooser-pane into splitter: */
        m_pSplitter->addWidget(m_pPaneOfChooser);
    }
}

void UIExtraDataManagerWindow::preparePaneData()
{
    /* Create data-pane: */
    m_pPaneOfData = new QWidget;
    AssertPtrReturnVoid(m_pPaneOfData);
    {
        /* Create layout: */
        QVBoxLayout *pLayout = new QVBoxLayout(m_pPaneOfData);
        AssertReturnVoid(pLayout && m_pPaneOfData->layout() &&
                         pLayout == m_pPaneOfData->layout());
        {
            /* Configure layout: */
            const int iL = qApp->style()->pixelMetric(QStyle::PM_LayoutLeftMargin) / 3;
            pLayout->setContentsMargins(iL, 0, 0, 0);
            /* Create data-filter: */
            m_pFilterOfData = new QLineEdit;
            {
                /* Configure data-filter: */
                m_pFilterOfData->setPlaceholderText("Search..");
                connect(m_pFilterOfData, &QLineEdit::textChanged,
                        this, &UIExtraDataManagerWindow::sltDataApplyFilter);
                /* Add data-filter into layout: */
                pLayout->addWidget(m_pFilterOfData);
            }
            /* Create data-view: */
            m_pViewOfData = new QTableView;
            AssertPtrReturnVoid(m_pViewOfData);
            {
                /* Create item-model: */
                m_pModelSourceOfData = new QStandardItemModel(0, 2, m_pViewOfData);
                AssertPtrReturnVoid(m_pModelSourceOfData);
                {
                    /* Create proxy-model: */
                    m_pModelProxyOfData = new QSortFilterProxyModel(m_pViewOfChooser);
                    AssertPtrReturnVoid(m_pModelProxyOfData);
                    {
                        /* Configure proxy-model: */
                        m_pModelProxyOfData->setSortCaseSensitivity(Qt::CaseInsensitive);
                        m_pModelProxyOfData->setFilterCaseSensitivity(Qt::CaseInsensitive);
                        m_pModelProxyOfData->setSourceModel(m_pModelSourceOfData);
                        m_pViewOfData->setModel(m_pModelProxyOfData);
                    }
                    /* Configure item-model: */
                    m_pModelSourceOfData->setHorizontalHeaderLabels(QStringList() << "Key" << "Value");
                }
                /* Configure data-view: */
                m_pViewOfData->setSortingEnabled(true);
                m_pViewOfData->setAlternatingRowColors(true);
                m_pViewOfData->setContextMenuPolicy(Qt::CustomContextMenu);
                m_pViewOfData->setSelectionMode(QAbstractItemView::ExtendedSelection);
                m_pViewOfData->setSelectionBehavior(QAbstractItemView::SelectRows);
                connect(m_pViewOfData, &QTableView::customContextMenuRequested,
                        this, &UIExtraDataManagerWindow::sltDataHandleCustomContextMenuRequested);
                QHeaderView *pVHeader = m_pViewOfData->verticalHeader();
                QHeaderView *pHHeader = m_pViewOfData->horizontalHeader();
                pVHeader->hide();
                pHHeader->setSortIndicator(0, Qt::AscendingOrder);
                pHHeader->resizeSection(0, qMin(300, pHHeader->width() / 3));
                pHHeader->setStretchLastSection(true);
                /* Add data-view into layout: */
                pLayout->addWidget(m_pViewOfData);
            }
        }
        /* Add data-pane into splitter: */
        m_pSplitter->addWidget(m_pPaneOfData);
    }
}

void UIExtraDataManagerWindow::prepareButtonBox()
{
    /* Create button-box: */
    m_pButtonBox = new QIDialogButtonBox;
    AssertPtrReturnVoid(m_pButtonBox);
    {
        /* Configure button-box: */
        m_pButtonBox->setStandardButtons(QDialogButtonBox::Help | QDialogButtonBox::Close);
        m_pButtonBox->button(QDialogButtonBox::Close)->setShortcut(Qt::Key_Escape);
        connect(m_pButtonBox, &QIDialogButtonBox::helpRequested, m_pButtonBox, &QIDialogButtonBox::sltHandleHelpRequest);
        connect(m_pButtonBox, &QIDialogButtonBox::rejected,      this, &UIExtraDataManagerWindow::close);
        /* Add button-box into main layout: */
        m_pMainLayout->addWidget(m_pButtonBox);
    }
}

void UIExtraDataManagerWindow::loadSettings()
{
    /* Load window geometry: */
    {
        const QRect geo = gEDataManager->extraDataManagerGeometry(this, m_pCenterWidget);
        LogRel2(("GUI: UIExtraDataManagerWindow: Restoring geometry to: Origin=%dx%d, Size=%dx%d\n",
                 geo.x(), geo.y(), geo.width(), geo.height()));
        restoreGeometry(geo);
    }

    /* Load splitter hints: */
    {
        m_pSplitter->setSizes(gEDataManager->extraDataManagerSplitterHints(this));
    }
}

void UIExtraDataManagerWindow::saveSettings()
{
    /* Save splitter hints: */
    {
        gEDataManager->setExtraDataManagerSplitterHints(m_pSplitter->sizes());
    }

    /* Save window geometry: */
    {
        const QRect geo = currentGeometry();
        LogRel2(("GUI: UIExtraDataManagerWindow: Saving geometry as: Origin=%dx%d, Size=%dx%d\n",
                 geo.x(), geo.y(), geo.width(), geo.height()));
        gEDataManager->setExtraDataManagerGeometry(geo, isCurrentlyMaximized());
    }
}

void UIExtraDataManagerWindow::cleanup()
{
    /* Save settings: */
    saveSettings();
}

void UIExtraDataManagerWindow::updateActionsAvailability()
{
    /* Is there something selected in chooser-view? */
    bool fChooserHasSelection = !m_pViewOfChooser->selectionModel()->selection().isEmpty();
    /* Is there something selected in data-view? */
    bool fDataHasSelection = !m_pViewOfData->selectionModel()->selection().isEmpty();

    /* Enable/disable corresponding actions: */
    m_pActionAdd->setEnabled(fChooserHasSelection);
    m_pActionDel->setEnabled(fChooserHasSelection && fDataHasSelection);
    m_pActionLoad->setEnabled(fChooserHasSelection);
    m_pActionSave->setEnabled(fChooserHasSelection);
}

QModelIndex UIExtraDataManagerWindow::chooserIndex(int iRow) const
{
    return m_pModelSourceOfChooser->index(iRow, 0);
}

QModelIndex UIExtraDataManagerWindow::currentChooserIndex() const
{
    return m_pViewOfChooser->currentIndex();
}

QUuid UIExtraDataManagerWindow::chooserID(int iRow) const
{
    return chooserIndex(iRow).data(Field_ID).toUuid();
}

QUuid UIExtraDataManagerWindow::currentChooserID() const
{
    return currentChooserIndex().data(Field_ID).toUuid();
}

QString UIExtraDataManagerWindow::chooserName(int iRow) const
{
    return chooserIndex(iRow).data(Field_Name).toString();
}

QString UIExtraDataManagerWindow::currentChooserName() const
{
    return currentChooserIndex().data(Field_Name).toString();
}

void UIExtraDataManagerWindow::addChooserItem(const QUuid &uID,
                                              const QString &strName,
                                              const QString &strOsTypeID,
                                              const int iPosition /* = -1 */)
{
    /* Create item: */
    QStandardItem *pItem = new QStandardItem;
    AssertPtrReturnVoid(pItem);
    {
        /* Which is NOT editable: */
        pItem->setEditable(false);
        /* Contains passed ID: */
        pItem->setData(uID, Field_ID);
        /* Contains passed name: */
        pItem->setData(strName, Field_Name);
        /* Contains passed OS Type ID: */
        pItem->setData(strOsTypeID, Field_OsTypeID);
        /* And designated as known/unknown depending on extra-data manager status: */
        pItem->setData(gEDataManager->contains(uID), Field_Known);
        /* If insert position defined: */
        if (iPosition != -1)
        {
            /* Insert this item at specified position: */
            m_pModelSourceOfChooser->insertRow(iPosition, pItem);
        }
        /* If insert position undefined: */
        else
        {
            /* Add this item as the last one: */
            m_pModelSourceOfChooser->appendRow(pItem);
        }
    }
}

void UIExtraDataManagerWindow::addChooserItemByMachine(const CMachine &machine,
                                                       const int iPosition /* = -1 */)
{
    /* Make sure VM is accessible: */
    if (!machine.isNull() && machine.GetAccessible())
        return addChooserItem(machine.GetId(), machine.GetName(), machine.GetOSTypeId(), iPosition);
}

void UIExtraDataManagerWindow::addChooserItemByID(const QUuid &uID,
                                                  const int iPosition /* = -1 */)
{
    /* Global ID? */
    if (uID == UIExtraDataManager::GlobalID)
        return addChooserItem(uID, QString("Global"), QString(), iPosition);

    /* Search for the corresponding machine by ID: */
    CVirtualBox vbox = gpGlobalSession->virtualBox();
    const CMachine machine = vbox.FindMachine(uID.toString());
    /* Make sure VM is accessible: */
    if (vbox.isOk() && !machine.isNull() && machine.GetAccessible())
        return addChooserItem(uID, machine.GetName(), machine.GetOSTypeId(), iPosition);
}

void UIExtraDataManagerWindow::makeSureChooserHaveCurrentIndexIfPossible()
{
    /* Make sure chooser have current-index if possible: */
    if (!m_pViewOfChooser->currentIndex().isValid())
    {
        /* Do we still have anything to select? */
        const QModelIndex firstIndex = m_pModelProxyOfChooser->index(0, 0);
        if (firstIndex.isValid())
            m_pViewOfChooser->setCurrentIndex(firstIndex);
    }
}

QModelIndex UIExtraDataManagerWindow::dataIndex(int iRow, int iColumn) const
{
    return m_pModelSourceOfData->index(iRow, iColumn);
}

QModelIndex UIExtraDataManagerWindow::dataKeyIndex(int iRow) const
{
    return dataIndex(iRow, 0);
}

QModelIndex UIExtraDataManagerWindow::dataValueIndex(int iRow) const
{
    return dataIndex(iRow, 1);
}

QString UIExtraDataManagerWindow::dataKey(int iRow) const
{
    return dataKeyIndex(iRow).data().toString();
}

QString UIExtraDataManagerWindow::dataValue(int iRow) const
{
    return dataValueIndex(iRow).data().toString();
}

void UIExtraDataManagerWindow::addDataItem(const QString &strKey,
                                           const QString &strValue,
                                           const int iPosition /* = -1 */)
{
    /* Prepare items: */
    QList<QStandardItem*> items;
    /* Create key item: */
    items << new QStandardItem(strKey);
    items.last()->setData(strKey, Qt::UserRole);
    AssertPtrReturnVoid(items.last());
    /* Create value item: */
    items << new QStandardItem(strValue);
    AssertPtrReturnVoid(items.last());
    /* If insert position defined: */
    if (iPosition != -1)
    {
        /* Insert these items as the row at the required position: */
        m_pModelSourceOfData->insertRow(iPosition, items);
    }
    /* If insert position undefined: */
    else
    {
        /* Add these items as the last one row: */
        m_pModelSourceOfData->appendRow(items);
    }
}

void UIExtraDataManagerWindow::sortData()
{
    /* Sort using current rules: */
    const QHeaderView *pHHeader = m_pViewOfData->horizontalHeader();
    const int iSortSection = pHHeader->sortIndicatorSection();
    const Qt::SortOrder sortOrder = pHHeader->sortIndicatorOrder();
    m_pModelProxyOfData->sort(iSortSection, sortOrder);
}
