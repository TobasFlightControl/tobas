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
void addWidgetCenter(QWidget* widget, QVBoxLayout* rows, int stretch = 0);

/* ウィジェットをレイアウトの中央に追加する． */
void addWidgetCenter(QWidget* widget, QHBoxLayout* cols, int stretch = 0);

/* サイズポリシー付きのスペーサを挿入する． */
void addSpacing(QVBoxLayout* rows, int height, QSizePolicy::Policy v_policy);

/* サイズポリシー付きのスペーサを挿入する． */
void addSpacing(QHBoxLayout* cols, int width, QSizePolicy::Policy h_policy);

/* 幅固定のQVBoxLayoutを作成する． */
QVBoxLayout* createFixedWidthQVBoxLayout(int width, QBoxLayout* parent);

/* 高さ固定のQVBoxLayoutを作成する． */
QHBoxLayout* createFixedHeightQHBoxLayout(int height, QBoxLayout* parent);

/* レイアウト内の全ての要素を削除する． */
void clearLayout(QLayout* layout);

/* 複数のウィジェットを縦に並べたウィジェットを作成する． */
QWidget* createVerticalWidgetsContainer(const std::vector<QWidget*>& widgets);

/* 複数のウィジェットを横に並べたウィジェットを作成する． */
QWidget* createHorizontalWidgetsContainer(const std::vector<QWidget*>& widgets);

/* ScrollWidgetを挟んでQVBoxLayoutを作成する． */
QVBoxLayout* createScrollableQVBoxLayout(QBoxLayout* parent);
}  // namespace qt
