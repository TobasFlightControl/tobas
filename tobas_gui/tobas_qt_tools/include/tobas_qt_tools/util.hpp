#pragma once

#include <QObject>
#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>

namespace qt
{
/* 子ウィジェットを再帰的に走査し，全てのシグナルをブロックする． */
void blockSignalsRec(QObject* obj, bool block);

/* ウィジェットをレイアウトの中央に追加する． */
void addWidgetCenter(QWidget* widget, QVBoxLayout* rows);

/* 幅固定のQVBoxLayoutを作成する． */
QVBoxLayout* createFixedWidthQVBoxLayout(int width, QBoxLayout* parent);

/* 高さ固定のQVBoxLayoutを作成する． */
QHBoxLayout* createFixedHeightQHBoxLayout(int height, QBoxLayout* parent);

/* レイアウト内の全てのウィジェットを削除する． */
void clearLayout(QLayout* layout);

/* 複数のウィジェットを縦に並べたウィジェットを作成する． */
QWidget* createVerticalWidgetsContainer(const std::vector<QWidget*>& widgets);

/* 複数のウィジェットを横に並べたウィジェットを作成する． */
QWidget* createHorizontalWidgetsContainer(const std::vector<QWidget*>& widgets);
}  // namespace qt
