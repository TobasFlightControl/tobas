#pragma once

#include <QObject>
#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>

namespace qt
{
/* 子ウィジェットを再帰的に走査し，全てのシグナルをブロックする． */
void blockSignalsRec(QObject* obj, bool block);

/* ウィジェットをレイアウトの中央に配置する． */
void placeCenter(QWidget* widget, QVBoxLayout* rows);

/* 幅固定のQVBoxLayoutを作成する． */
QVBoxLayout* createFixedWidthQVBoxLayout(int width, QBoxLayout* parent);

/* 高さ固定のQVBoxLayoutを作成する． */
QHBoxLayout* createFixedHeightQHBoxLayout(int width, QBoxLayout* parent);
}  // namespace qt
